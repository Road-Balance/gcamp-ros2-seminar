# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

count = 0

class NodeClass(Node):
    def __init__(self):
        super().__init__("node_name")

def main(args=None):
    rclpy.init(args=args)

    node = NodeClass()
    node.get_logger().info('\n==== Hello ROS 2 ====')
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()