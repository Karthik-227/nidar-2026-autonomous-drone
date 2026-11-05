#!/usr/bin/env python3
"""Carrot-on-path follower for a holonomic drone. Replaces DWB's output.
Reads /plan (map frame) + TF + /scan, publishes body-frame Twist, yaw rate 0."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        # Must run before any subscription/publisher/timer exists: it spins
        # the node briefly to check for a duplicate node name. Catches the
        # failure mode that actually caused the drone to oscillate on 2 Nov
        # 2026 - a leftover path_follower.py from an earlier test session
        # stayed alive (nidar-clean.sh's kill patterns only match installed
        # lib/nidar_sim/ executables, not a bare `python3 path_follower.py`
        # run from this directory) and kept publishing to /cmd_vel_follower
        # alongside a freshly-started one. cmd_vel_bridge just forwards
        # whatever arrives, so two independent carrot-followers fought over
        # the output every ~100ms - no replanning or costmap issue involved,
        # just two controllers disagreeing. Refuses to start rather than
        # silently repeat that.
        self._assert_singleton()
        p = self.declare_parameters('', [
            ('lookahead', 0.30), ('gain', 1.0), ('v_max', 0.30),
            ('guard_dist', 0.35), ('guard_half_angle_deg', 30.0),
            ('path_timeout', 2.0), ('scan_timeout', 1.0), ('end_tolerance', 0.10),
            ('cmd_topic', '/cmd_vel_follower')])
        g = lambda n: self.get_parameter(n).value
        self.lookahead, self.gain, self.v_max = g('lookahead'), g('gain'), g('v_max')
        self.guard, self.guard_ang = g('guard_dist'), math.radians(g('guard_half_angle_deg'))
        self.path_timeout, self.scan_timeout, self.end_tol = g('path_timeout'), g('scan_timeout'), g('end_tolerance')

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(Path, '/plan', self.on_path, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.pub = self.create_publisher(Twist, g('cmd_topic'), 10)

        self.pts, self.path_time, self.idx = [], None, 0
        self.scan, self.scan_time = None, None
        self.state = None
        self.create_timer(0.1, self.tick)

    def _assert_singleton(self):
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        peers = [n for n, _ in self.get_node_names_and_namespaces()
                 if n == self.get_name()]
        if len(peers) > 1:
            self.get_logger().fatal(
                f'{len(peers)} nodes named "{self.get_name()}" are on the ROS '
                f'graph. Both publish velocity to /cmd_vel_follower from their '
                f'own independent carrot-tracking state, so the drone gets '
                f'alternating/conflicting commands from each - this is what '
                f'caused the oscillation investigated on 2 Nov 2026. '
                f'Refusing to start a duplicate.\n'
                f'  Find it:  ros2 node list | grep {self.get_name()}\n'
                f'  Or by PID: ps -eo pid,cmd | grep path_follower.py')
            raise SystemExit(1)

    def on_path(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warn(f"path frame '{msg.header.frame_id}' != map; ignoring", throttle_duration_sec=5.0)
            return
        self.pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_time, self.idx = self.get_clock().now(), 0

    def on_scan(self, msg):
        self.scan, self.scan_time = msg, self.get_clock().now()

    def pose(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def carrot(self, x, y):
        """Nearest path point at/after last index, then walk lookahead metres forward."""
        n = len(self.pts)
        self.idx = min(range(self.idx, n), key=lambda i: (self.pts[i][0] - x) ** 2 + (self.pts[i][1] - y) ** 2)
        d, j = 0.0, self.idx
        while j + 1 < n and d < self.lookahead:
            d += math.dist(self.pts[j], self.pts[j + 1])
            j += 1
        return self.pts[j], j == n - 1

    def blocked(self, heading_body):
        s = self.scan
        for i, r in enumerate(s.ranges):
            if not (s.range_min < r < s.range_max) or r >= self.guard:
                continue
            a = s.angle_min + i * s.angle_increment
            if abs(math.atan2(math.sin(a - heading_body), math.cos(a - heading_body))) <= self.guard_ang:
                return True
        return False

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(s)
            self.state = s

    def hold(self, why):
        self.pub.publish(Twist())
        self.set_state(f'HOLD: {why}')

    def tick(self):
        now = self.get_clock().now()
        if not self.pts or now - self.path_time > Duration(seconds=self.path_timeout):
            return self.hold('no fresh path')
        if self.scan is None or now - self.scan_time > Duration(seconds=self.scan_timeout):
            return self.hold('no fresh scan')
        p = self.pose()
        if p is None:
            return self.hold('no map->base_link')
        x, y, yaw = p
        (cx, cy), at_end = self.carrot(x, y)
        ex, ey = cx - x, cy - y
        dist = math.hypot(ex, ey)
        if at_end and dist < self.end_tol:
            return self.hold('at path end')
        vx, vy = self.gain * ex, self.gain * ey
        v = math.hypot(vx, vy)
        if v > self.v_max:
            vx, vy = vx * self.v_max / v, vy * self.v_max / v
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * vx + s * vy, -s * vx + c * vy          # map -> body (FLU)
        if self.blocked(math.atan2(by, bx)):
            return self.hold(f'lidar < {self.guard:.2f} m ahead')
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = bx, by
        self.pub.publish(cmd)
        self.set_state(f'TRACKING idx={self.idx}/{len(self.pts)}')


def main():
    rclpy.init()
    rclpy.spin(PathFollower())


if __name__ == '__main__':
    main()
