# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistSubNode(Node):
    def __init__(self):
        super().__init__("twist_sub_node")
        self.get_logger().info(f"TwistSubNode Created at {self.get_clock().now().to_msg().sec}")
        self.twist_subscriber = self.create_subscription(Twist, 'twist_topic', self.sub_callback, 10)
        # self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        self.get_logger().info(f"Linear X velocity : {msg.linear.x}")


def main(args=None):
    rclpy.init(args=args)

    t_pub_node = TwistSubNode()
    rclpy.spin(t_pub_node)
    t_pub_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()