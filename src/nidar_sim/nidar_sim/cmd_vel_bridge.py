import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

import math
import tf2_ros
from tf2_ros import TransformException

def _qos(reliability: str, depth: int) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.history = QoSHistoryPolicy.KEEP_LAST
    qos.durability = QoSDurabilityPolicy.VOLATILE
    qos.reliability = (
        QoSReliabilityPolicy.RELIABLE
        if reliability.lower() == 'reliable'
        else QoSReliabilityPolicy.BEST_EFFORT
    )
    return qos

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Must run before any subscription/timer exists: it spins the node
        # (rclpy.spin_once in a loop) to check for a duplicate node name, and
        # a callback firing during that spin would hit attributes this
        # __init__ hasn't set yet. Previously called after the /cmd_vel
        # subscription was already live, which crashed with
        # AttributeError: 'CmdVelBridge' object has no attribute
        # '_last_was_zero' whenever a publisher on in_topic (e.g.
        # path_follower.py) was already active before this node started.
        self._assert_singleton()

        self.declare_parameter('in_topic', '/cmd_vel')
       
        self.declare_parameter('out_topic', '/ap/v1/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('input_stamped', False)
        self.declare_parameter('in_reliability', 'best_effort')
        self.declare_parameter('out_reliability', 'best_effort')
        self.declare_parameter('queue_depth', 10)
        
        self.declare_parameter('stop_repeat_sec', 1.0)
        
        self.declare_parameter('max_linear_vel', 1.50)
        self.declare_parameter('max_angular_vel', 1.5)
        
        

        self.declare_parameter('negate_lateral_for_frd', True)
        self.declare_parameter('command_frame', 'base_link')
        self.declare_parameter('yaw_reference_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('command_timeout_sec', 0.3)
        self.declare_parameter('target_altitude_m', 1.17)
        self.declare_parameter('altitude_gain', 0.8)
        self.declare_parameter('max_climb_rate', 0.4)
        self.declare_parameter('odom_topic', '/odometry')

        self.frame_id = self.get_parameter('frame_id').value
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        input_stamped = self.get_parameter('input_stamped').value
        depth = self.get_parameter('queue_depth').value
        self.max_linear = float(self.get_parameter('max_linear_vel').value)
        self.stop_repeat_sec = float(
            self.get_parameter('stop_repeat_sec').value)
        self.max_angular = float(self.get_parameter('max_angular_vel').value)

        self.pub = self.create_publisher(
            TwistStamped, out_topic,
            _qos(self.get_parameter('out_reliability').value, depth))

        in_qos = _qos(self.get_parameter('in_reliability').value, depth)
        if input_stamped:
            self.sub = self.create_subscription(
                TwistStamped, in_topic, self._cb_stamped, in_qos)
        else:
            self.sub = self.create_subscription(
                Twist, in_topic, self._cb_twist, in_qos)

        self.target_altitude_m = self.get_parameter('target_altitude_m').value
        self.altitude_gain = self.get_parameter('altitude_gain').value
        self.max_climb_rate = self.get_parameter('max_climb_rate').value
        self._altitude = None
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_cb, 10)

        self.negate_lateral = bool(
            self.get_parameter('negate_lateral_for_frd').value)
        self.command_frame = str(self.get_parameter('command_frame').value)

        self.yaw_ref_frame = str(self.get_parameter('yaw_reference_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self._tf_buffer = None
        self._yaw_warned = False
        if self.command_frame == 'map':
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            self.frame_id = 'map'

        self._clamped_warned = False
        self._last_was_zero = True
        self._stop_until = None
        self._idle_logged = False
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value)
        self._last_cmd_time = None
        self._deadman_fired = False
        if self.command_timeout_sec > 0.0:
            self.create_timer(0.05, self._deadman_check)
        self.get_logger().info(
            f"cmd_vel bridge up: {in_topic} "
            f"({'TwistStamped' if input_stamped else 'Twist'}) -> {out_topic} "
            f"(TwistStamped, frame '{self.frame_id}'). "
            f"Clamps: linear {self.max_linear} m/s, angular {self.max_angular} rad/s."
        )

    def _assert_singleton(self):
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        peers = [n for n, _ in self.get_node_names_and_namespaces()
                 if n == self.get_name()]
        if len(peers) > 1:
            self.get_logger().fatal(
                f'{len(peers)} nodes named "{self.get_name()}" are on the ROS '
                f'graph. Both publish to the autopilot, and their idle-quiet '
                f'state is independent - so one keeps ArduPilot pinned at zero '
                f'velocity (which sinks the drone and blocks takeoff) while '
                f'the other reports that it went quiet. Refusing to start a '
                f'duplicate.\n'
                f'  Find them:  ros2 node list | grep {self.get_name()}\n'
                f'  Note:       nav2.launch.py ALREADY starts one. Either use '
                f'that, or launch it with bridge:=false and run this by hand - '
                f'not both.')
            raise SystemExit(1)

    def _clamp(self, twist: Twist) -> Twist:
        if self.max_linear > 0.0:
            speed = math.hypot(twist.linear.x, twist.linear.y)
            if speed > self.max_linear:
                scale = self.max_linear / speed
                twist.linear.x *= scale
                twist.linear.y *= scale
                self._warn_clamped()
            if abs(twist.linear.z) > self.max_linear:
                twist.linear.z = math.copysign(self.max_linear, twist.linear.z)
                self._warn_clamped()
        if self.max_angular > 0.0:
            for axis in ('x', 'y', 'z'):
                v = getattr(twist.angular, axis)
                if abs(v) > self.max_angular:
                    setattr(twist.angular, axis,
                            self.max_angular if v > 0 else -self.max_angular)
                    self._warn_clamped()
        return twist

    def _warn_clamped(self):
        if not self._clamped_warned:
            self._clamped_warned = True
            self.get_logger().warn(
                'Clamped an incoming velocity command to the configured limits. '
                'If this repeats, retune Nav2 (velocity_smoother.max_velocity / '
                'FollowPath.desired_linear_vel) rather than raising the clamp - '
                'the clamp is a backstop, not a speed setting.')

    def _odom_cb(self, msg: Odometry):
        self._altitude = msg.pose.pose.position.z

    def _altitude_correction(self):
        if self.target_altitude_m <= 0.0 or self._altitude is None:
            return None
        err = self.target_altitude_m - self._altitude
        vz = self.altitude_gain * err
        return max(-self.max_climb_rate, min(self.max_climb_rate, vz))

    @staticmethod
    def _is_zero(twist: Twist, eps=1e-4):
        return (abs(twist.linear.x) < eps and abs(twist.linear.y) < eps
                and abs(twist.linear.z) < eps and abs(twist.angular.x) < eps
                and abs(twist.angular.y) < eps and abs(twist.angular.z) < eps)

    def _publish(self, twist: Twist):
        
        self._last_cmd_time = self.get_clock().now()
        self._deadman_fired = False
        zero = self._is_zero(twist)
       
        now = self.get_clock().now()
        if zero and self._last_was_zero:
            if self._stop_until is None or now >= self._stop_until:
                return
        if zero:
            if not self._last_was_zero:
                self._stop_until = now + Duration(seconds=self.stop_repeat_sec)
            self._last_was_zero = True
            if not self._idle_logged:
                self._idle_logged = True
                self.get_logger().info(
                    f'Nav2 idle - repeating the stop for {self.stop_repeat_sec:.1f}s, '
                    'then going quiet so '
                    'ArduPilot holds position instead of being pinned at '
                    'zero velocity.')
        else:
            self._last_was_zero = False
            self._stop_until = None
            self._idle_logged = False

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = self._clamp(twist)
        if self.command_frame == 'map':
            if not self._to_enu(out.twist):
                return
        elif self.negate_lateral:
            out.twist.linear.y = -out.twist.linear.y
        vz = self._altitude_correction()
        if vz is not None:
            out.twist.linear.z = vz
        self.pub.publish(out)

    def _body_yaw(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.yaw_ref_frame, self.robot_base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            return None
        q = t.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _to_enu(self, twist: Twist):
        yaw = self._body_yaw()
        if yaw is None:
            if not self._yaw_warned:
                self._yaw_warned = True
                self.get_logger().error(
                    f'command_frame=map but {self.yaw_ref_frame}->'
                    f'{self.robot_base_frame} is not available, so a body-frame '
                    f'twist cannot be rotated into ENU. Holding commands rather '
                    f'than sending one in the wrong direction.')
            return False
        self._yaw_warned = False
        vx, vy = twist.linear.x, twist.linear.y
        c, s_ = math.cos(yaw), math.sin(yaw)
        twist.linear.x = vx * c - vy * s_
        twist.linear.y = vx * s_ + vy * c
        return True

    def _deadman_check(self):
        if self._last_was_zero or self._last_cmd_time is None:
            return
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age < self.command_timeout_sec:
            return

        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = self.frame_id
        self.pub.publish(stop)
        self._last_was_zero = True
        self._stop_until = (self.get_clock().now()
                            + Duration(seconds=self.stop_repeat_sec))
        if not self._deadman_fired:
            self._deadman_fired = True
            self.get_logger().error(
                f'/cmd_vel went silent for {age:.2f}s while the drone was '
                f'still moving - sent a stop. ArduPilot holds the last '
                f'commanded velocity until its own GUIDED timeout (~3 s), so '
                f'without this the vehicle keeps flying at cruise speed. The '
                f'usual cause is controller_server aborting mid-goal, e.g. '
                f'"RegulatedPurePursuitController detected collision ahead!".')

    def _cb_twist(self, msg: Twist):
        self._publish(msg)

    def _cb_stamped(self, msg: TwistStamped):
        self._publish(msg.twist)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop = TwistStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            stop.header.frame_id = node.frame_id
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
