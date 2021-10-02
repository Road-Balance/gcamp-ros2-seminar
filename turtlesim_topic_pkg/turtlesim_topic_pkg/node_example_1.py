# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=None)

    node = Node('my_node_name')
    
    rclpy.spin(node)
    
    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()