# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node = Node("node_name")
    node.get_logger().info('\n==== Hello ROS 2 ====')
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()