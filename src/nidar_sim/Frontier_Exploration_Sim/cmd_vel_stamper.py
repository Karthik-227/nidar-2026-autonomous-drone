#the bridge for Nav 2 to Ardupilot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/ap/v1/cmd_vel')
        frame = self.get_parameter('frame_id').value
        self._frame = frame
        self._pub = self.create_publisher(
            TwistStamped, self.get_parameter('out_topic').value, 10)
        self._sub = self.create_subscription(
            Twist, self.get_parameter('in_topic').value, self._cb, 10)
        self.get_logger().info(
            f"Bridging {self.get_parameter('in_topic').value} (Twist) -> "
            f"{self.get_parameter('out_topic').value} (TwistStamped, frame '{frame}')")

    def _cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        out.twist.linear.x = msg.linear.x
        out.twist.linear.y = -msg.linear.y   # ROS FLU (+y left) -> ArduPilot FRD (+y right)
        out.twist.linear.z = msg.linear.z
        out.twist.angular.x = msg.angular.x
        out.twist.angular.y = msg.angular.y
        out.twist.angular.z = msg.angular.z
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
