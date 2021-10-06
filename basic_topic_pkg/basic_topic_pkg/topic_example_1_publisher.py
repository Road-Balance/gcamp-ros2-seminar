# !/usr/bin/env/ python3

import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TwistPubNode(Node):
    def __init__(self):
        super().__init__("twist_pub_node")

        self.get_logger().info(f"TwistPubNode Created at {self.get_clock().now().to_msg().sec}")
        # self.twist_publisher = self.create_publisher(Twist, "twist_topic", 10)
        self.twist_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = random.random()
        msg.angular.z = random.random()
        self.get_logger().info(f"Linear X velocity : {msg.linear.x} / Angular Z velocity : {msg.angular.z}")

        self.twist_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    twist_pub_node = TwistPubNode()
    rclpy.spin(twist_pub_node)
    twist_pub_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()