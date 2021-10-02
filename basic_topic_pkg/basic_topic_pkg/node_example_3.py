# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

count = 0

def timer_callback():
    global count
    count += 1
    print(f"==== Hello ROS 2 : {count}====")
    # How can I use logger without globalization ?
    # node.get_logger().info('\n==== Hello ROS 2 ====')


def main(args=None):
    rclpy.init(args=args)

    node = Node("node_name")
    node.create_timer(0.2, timer_callback)

    while True:
        rclpy.spin_once(node)

    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()