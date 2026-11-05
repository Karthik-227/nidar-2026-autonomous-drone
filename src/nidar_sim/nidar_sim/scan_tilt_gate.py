import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, LaserScan

import tf2_ros
from tf2_ros import TransformException

class ScanTiltGate(Node):
    def __init__(self):
        super().__init__('scan_tilt_gate')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_safe')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('max_safe_tilt_deg', 8.0)
        self.declare_parameter('tilt_release_margin_deg', 1.5)
        self.declare_parameter('tf_timeout_sec', 0.05)

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.max_safe_tilt_deg = self.get_parameter('max_safe_tilt_deg').value
        self.tilt_release_margin_deg = self.get_parameter(
            'tilt_release_margin_deg').value
        self.tf_timeout_sec = self.get_parameter('tf_timeout_sec').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._imu_tilt_deg = None
        self._imu_stamp = None
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_cb, qos_profile_sensor_data)

        self._ungated_scans = 0
        self.create_timer(10.0, self._report_if_ungated)

        self.last_good_scan = None
        self._holding = False
        self._tf_warned_once = False

        self.sub = self.create_subscription(
            LaserScan, self.input_scan_topic, self._scan_cb,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LaserScan, self.output_scan_topic, qos_profile_sensor_data)

        self.get_logger().error(
            'scan_tilt_gate is DEPRECATED and should not be running. It '
            'holds the last level scan while the airframe leans, which '
            'starves slam_toolbox (a repeated timestamp is rejected by '
            'minimum_time_interval) and feeds Nav2 scans placed at a pose the '
            'drone has already left. Use scan_stabilizer instead - same topic '
            '(/scan_safe), but it corrects the scan geometry rather than '
            'withholding it:  ros2 launch nidar_sim slam.launch.py')
        self.get_logger().info(
            f"scan_tilt_gate started: '{self.input_scan_topic}' -> "
            f"'{self.output_scan_topic}', gating on {self.reference_frame}->"
            f"<scan frame>, max_safe_tilt_deg={self.max_safe_tilt_deg:.1f}.")

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        if (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 0.0):
            return
        world_z_z = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
        self._imu_tilt_deg = math.degrees(math.acos(world_z_z))
        self._imu_stamp = self.get_clock().now()

    def _report_if_ungated(self):
        if self._imu_tilt_deg is not None:
            return
        if self._ungated_scans == 0:
            return
        self.get_logger().error(
            f'TILT GATE IS NOT GATING. {self._ungated_scans} scans have been '
            f'forwarded unfiltered because no tilt source is available - no '
            f'IMU on "{self.get_parameter("imu_topic").value}" and no '
            f'{self.reference_frame}->scan transform. Every downstream '
            f'consumer (SLAM, both costmaps) is receiving raw scans including '
            f'corrupted ones, which is what paints phantom obstacles and '
            f'erases real walls. Check: ros2 topic hz '
            f'{self.get_parameter("imu_topic").value}')
        self._ungated_scans = 0

    def _tilt_deg(self, scan_frame_id, stamp):
        if self._imu_tilt_deg is not None:
            return self._imu_tilt_deg

        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, scan_frame_id, stamp,
                timeout=Duration(seconds=self.tf_timeout_sec))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.reference_frame, scan_frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec))
            except TransformException as ex:
                if not self._tf_warned_once:
                    self._tf_warned_once = True
                    self.get_logger().warn(
                        f'TF lookup failed ({self.reference_frame}->'
                        f'{scan_frame_id} not ready yet?): {ex}. Passing '
                        f'scans through unfiltered until TF is available '
                        f'(logged once).')
                return None

        q = tf.transform.rotation
        world_z_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        world_z_z = max(-1.0, min(1.0, world_z_z))
        return math.degrees(math.acos(world_z_z))

    def _scan_cb(self, msg: LaserScan):
        tilt_deg = self._tilt_deg(msg.header.frame_id, msg.header.stamp)

        if tilt_deg is None:
            self._ungated_scans += 1
            self.pub.publish(msg)
            return

        release_below = self.max_safe_tilt_deg - self.tilt_release_margin_deg
        threshold = release_below if self._holding else self.max_safe_tilt_deg

        if tilt_deg <= threshold:
            if self._holding:
                self.get_logger().info(
                    f'Tilt back to {tilt_deg:.1f} deg (<= '
                    f'{release_below:.1f} deg release threshold) - resuming '
                    f'live scans.')
                self._holding = False
            self.last_good_scan = msg
            self.pub.publish(msg)
            return

        if not self._holding:
            self.get_logger().warn(
                f'Tilt {tilt_deg:.1f} deg exceeds max_safe_tilt_deg='
                f'{self.max_safe_tilt_deg:.1f} deg - holding last '
                f'known-good scan instead of forwarding a corrupted one.')
            self._holding = True

        if self.last_good_scan is not None:
            self.pub.publish(self.last_good_scan)
        else:
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanTiltGate()
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
