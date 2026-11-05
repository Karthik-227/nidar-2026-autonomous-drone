import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, LaserScan

import tf2_ros
from tf2_ros import TransformException

def _quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])

def _matrix_to_quat(m):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w

def _yaw_only(m):
    yaw = math.atan2(m[1, 0], m[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

class ScanStabilizer(Node):
    def __init__(self):
        super().__init__('scan_stabilizer')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_safe')
        self.declare_parameter('stabilized_frame', 'base_scan_level')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('max_hit_dz_m', 0.75)
        self.declare_parameter('min_hit_height_agl_m', 0.25)
        self.declare_parameter('max_hit_height_agl_m', 2.20)
        self.declare_parameter('ground_height_m', 0.0)
        self.declare_parameter('max_clear_elev_deg', 8.0)
        self.declare_parameter('passthrough_tilt_deg', 0.3)

        self.declare_parameter('tf_timeout_sec', 0.15)
        self.declare_parameter('report_period_sec', 10.0)

        g = lambda n: self.get_parameter(n).value
        self.in_topic = g('input_scan_topic')
        self.out_topic = g('output_scan_topic')
        self.stab_frame = g('stabilized_frame')
        self.base_frame = g('robot_base_frame')
        self.ref_frame = g('reference_frame')
        self.publish_tf = bool(g('publish_tf'))
        self.max_hit_dz = float(g('max_hit_dz_m'))
        self.min_hit_agl = float(g('min_hit_height_agl_m'))
        self.max_hit_agl = float(g('max_hit_height_agl_m'))
        self.ground_z = float(g('ground_height_m'))
        self.max_clear_elev = math.radians(float(g('max_clear_elev_deg')))
        self.passthrough_tilt = math.radians(float(g('passthrough_tilt_deg')))
        self.tf_timeout = float(g('tf_timeout_sec'))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._imu_quat = None
        self.create_subscription(Imu, g('imu_topic'), self._imu_cb,
                                 qos_profile_sensor_data)

        self.pub = self.create_publisher(LaserScan, self.out_topic,
                                         qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, self.in_topic,
                                            self._scan_cb,
                                            qos_profile_sensor_data)

        self._geom_key = None
        self._dirs = None
        self._bins_per_turn = None

        self._stats = dict(scans=0, hits=0, cleared=0, dropped=0,
                           ground=0, ceiling=0, oblique=0, tilt_sum=0.0,
                           tilt_max=0.0, unstabilised=0)
        self.create_timer(float(g('report_period_sec')), self._report)

        self.get_logger().info(
            f"scan_stabilizer up: {self.in_topic} -> {self.out_topic} "
            f"(frame '{self.stab_frame}'). Re-projects every ray to its true "
            f"horizontal bearing/range using {self.ref_frame}->"
            f"{self.base_frame} attitude; returns further than "
            f"{self.max_hit_dz:.2f} m off the scan plane, within "
            f"{self.min_hit_agl:.2f} m of the ground, or at/above "
            f"{self.max_hit_agl:.2f} m (the net) become NaN rather than "
            f"ranges - so a beam that skims over a wall or clips the net can "
            f"no longer paint an obstacle or ray-trace a wall away.")

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        if (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 0.0):
            return
        self._imu_quat = (q.x, q.y, q.z, q.w)

    def _lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout))
            except TransformException:
                return None

    def _pose(self, scan_frame_id, stamp):
        tf_bs = self._lookup(self.base_frame, scan_frame_id, stamp)
        if tf_bs is None:
            t_bs = np.zeros(3)
            R_bs = np.eye(3)
        else:
            tr = tf_bs.transform.translation
            q = tf_bs.transform.rotation
            t_bs = np.array([tr.x, tr.y, tr.z])
            R_bs = _quat_to_matrix(q.x, q.y, q.z, q.w)

        tf_ob = self._lookup(self.ref_frame, self.base_frame, stamp)
        if tf_ob is not None:
            q = tf_ob.transform.rotation
            R_ob = _quat_to_matrix(q.x, q.y, q.z, q.w)
            base_z = tf_ob.transform.translation.z
            sensor_z = base_z + float((R_ob @ t_bs)[2])
        elif self._imu_quat is not None:
            R_ob = _quat_to_matrix(*self._imu_quat)
            sensor_z = None
        else:
            return None

        return R_ob @ R_bs, t_bs, R_ob, sensor_z

    def _ensure_geometry(self, msg: LaserScan):
        n = len(msg.ranges)
        key = (n, msg.angle_min, msg.angle_increment)
        if key == self._geom_key:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        self._dirs = np.vstack((np.cos(angles), np.sin(angles), np.zeros(n)))

        inc = msg.angle_increment
        self._bins_per_turn = None
        if inc > 0.0:
            per_turn = int(round(2.0 * math.pi / inc))
            if abs((msg.angle_max - msg.angle_min)) >= 2.0 * math.pi - 1.5 * inc:
                self._bins_per_turn = per_turn
        self._geom_key = key
        self.get_logger().info(
            f'Scan geometry: {n} rays, {math.degrees(msg.angle_min):.1f} to '
            f'{math.degrees(msg.angle_max):.1f} deg, '
            f'{math.degrees(inc):.3f} deg/ray, '
            f'{"full circle (wraps)" if self._bins_per_turn else "sector"}.')

    def _bin(self, az, n, angle_min, inc):
        idx = np.rint((az - angle_min) / inc).astype(np.int64)
        if self._bins_per_turn:
            idx = np.mod(idx, self._bins_per_turn)
        keep = (idx >= 0) & (idx < n)
        return idx, keep

    def _scan_cb(self, msg: LaserScan):
        self._ensure_geometry(msg)
        pose = self._pose(msg.header.frame_id, msg.header.stamp)

        if pose is None:
            self._stats['unstabilised'] += 1
            self.pub.publish(msg)
            return

        R_ws, t_bs, R_wb, sensor_z = pose
        tilt = math.acos(max(-1.0, min(1.0, R_ws[2, 2])))
        self._stats['scans'] += 1
        self._stats['tilt_sum'] += math.degrees(tilt)
        self._stats['tilt_max'] = max(self._stats['tilt_max'],
                                      math.degrees(tilt))

        R_wl = _yaw_only(R_ws)
        if self.publish_tf:
            self._broadcast(msg.header.stamp, R_wb, R_wl, t_bs)

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.stab_frame
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        if tilt <= self.passthrough_tilt:
            out.ranges = list(msg.ranges)
            self.pub.publish(out)
            return

        n = len(msg.ranges)
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        eps = max(1e-3, 1e-4 * msg.range_max)
        is_hit = np.isfinite(ranges) & (ranges >= msg.range_min) & \
            (ranges < msg.range_max - eps)

        d = (R_wl.T @ R_ws) @ self._dirs

        result = np.full(n, np.nan, dtype=np.float32)

        if np.any(is_hit):
            p = d[:, is_hit] * ranges[is_hit]
            rh = np.hypot(p[0], p[1])
            dz = p[2]

            ok = (np.abs(dz) <= self.max_hit_dz) & (rh >= msg.range_min)
            n_oblique = int(np.count_nonzero(~(np.abs(dz) <= self.max_hit_dz)))
            if sensor_z is not None:
                above_ground = (sensor_z + dz) >= (self.ground_z + self.min_hit_agl)
                self._stats['ground'] += int(np.count_nonzero(~above_ground))
                ok &= above_ground
                if self.max_hit_agl > 0.0:
                    below_ceiling = (sensor_z + dz) <= (self.ground_z + self.max_hit_agl)
                    self._stats['ceiling'] += int(np.count_nonzero(~below_ceiling))
                    ok &= below_ceiling
            self._stats['oblique'] += n_oblique

            if np.any(ok):
                az = np.arctan2(p[1][ok], p[0][ok])
                idx, keep = self._bin(az, n, msg.angle_min, msg.angle_increment)
                idx, r_ok = idx[keep], rh[ok][keep]
                acc = np.full(n, np.inf)
                np.minimum.at(acc, idx, r_ok)
                hit_bins = np.isfinite(acc)
                result[hit_bins] = acc[hit_bins]
                self._stats['hits'] += int(np.count_nonzero(hit_bins))

        no_ret = ~is_hit
        if np.any(no_ret):
            elev = np.arcsin(np.clip(d[2][no_ret], -1.0, 1.0))
            level_enough = np.abs(elev) <= self.max_clear_elev
            self._stats['dropped'] += int(np.count_nonzero(~level_enough))
            if np.any(level_enough):
                sub = np.nonzero(no_ret)[0][level_enough]
                az = np.arctan2(d[1][sub], d[0][sub])
                idx, keep = self._bin(az, n, msg.angle_min, msg.angle_increment)
                idx = idx[keep]
                free = idx[np.isnan(result[idx])]
                result[free] = np.inf
                self._stats['cleared'] += int(free.size)

        if self._bins_per_turn and n == self._bins_per_turn + 1:
            result[n - 1] = result[0]

        out.ranges = result.tolist()
        self.pub.publish(out)

    def _broadcast(self, stamp, R_wb, R_wl, t_bs):
        R = R_wb.T @ R_wl
        qx, qy, qz, qw = _matrix_to_quat(R)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.stab_frame
        tf.transform.translation.x = float(t_bs[0])
        tf.transform.translation.y = float(t_bs[1])
        tf.transform.translation.z = float(t_bs[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _report(self):
        s = self._stats
        if s['unstabilised']:
            self.get_logger().error(
                f"NOT STABILISING. {s['unstabilised']} scans forwarded raw - "
                f"no {self.ref_frame}->{self.base_frame} transform and no IMU "
                f"orientation. Downstream is getting tilted scans, which is "
                f"what paints phantom obstacles and erases real walls. Check: "
                f"ros2 run tf2_ros tf2_echo {self.ref_frame} {self.base_frame}")
        if s['scans']:
            total = (s['hits'] + s['cleared'] + s['dropped'] + s['oblique']
                     + s['ground'] + s['ceiling'])
            withheld = (s['dropped'] + s['oblique'] + s['ground']
                        + s['ceiling']) / s['scans']
            self.get_logger().info(
                f"stabilised {s['scans']} scans | mean tilt "
                f"{s['tilt_sum'] / s['scans']:.1f} deg (peak "
                f"{s['tilt_max']:.1f}) | per scan: {s['hits'] / s['scans']:.0f} "
                f"obstacle, {s['cleared'] / s['scans']:.0f} clear, "
                f"{withheld:.0f} withheld ({s['ground']} ground, "
                f"{s['ceiling']} net/ceiling, {s['oblique']} off-plane, "
                f"{s['dropped']} tilted-miss of {total} rays)")
        self._stats.update(scans=0, hits=0, cleared=0, dropped=0, ground=0,
                           ceiling=0, oblique=0, tilt_sum=0.0, tilt_max=0.0,
                           unstabilised=0)

def main(args=None):
    rclpy.init(args=args)
    node = ScanStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
