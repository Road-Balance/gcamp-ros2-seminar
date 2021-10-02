# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

count = 0

def timer_callback():
    global count
    count += 1
    print(f"==== Hello ROS 2 {count}====")

def main(args=None):
    rclpy.init(args=None)

    node = Node('my_node_name')
    # node.create_timer(0.2, timer_callback)
    node.spin()
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# class MyNode(Node):
#     def __init__(self):
#         super().__init__('my_node_name')
#         self.create_timer(0.2, self.timer_callback)
#     def timer_callback(self):
#         self.get_logger().info("Hello ROS2")
# def main(args=None):
#     rclpy.init(args=args)
#     node = MyNode()
#     rclpy.spin(node)
#     rclpy.shutdown()
# if __name__ == '__main__':
#     main()