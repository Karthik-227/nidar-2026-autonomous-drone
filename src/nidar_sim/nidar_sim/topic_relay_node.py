import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped

class TopicRelayNode(Node):
    def __init__(self):
        super().__init__('topic_relay_node')
        self.declare_parameter('frame_id', 'base_link')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', qos)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info("Topic relay node created!")

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('frame_id').value
        out.twist = msg
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
