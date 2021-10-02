# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

count = 0

class NodeClass(Node):
    def __init__(self):
        super().__init__("node_name")
        self.create_timer(0.2, self.timer_callback)

        self.count = 1

    def timer_callback(self):
        self.get_logger().info(f"==== Hello ROS 2 : {self.count}====")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)

    node = NodeClass()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()