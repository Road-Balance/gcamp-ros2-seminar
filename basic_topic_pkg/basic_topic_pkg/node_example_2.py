# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

count = 0

def timer_callback():
    global count
    count += 1
    print(f"==== Hello ROS 2 : {count}====")


def main(args=None):
    rclpy.init(args=args)

    node = Node("node_name")
    node.create_timer(0.2, timer_callback)
    rclpy.spin(node)

    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()