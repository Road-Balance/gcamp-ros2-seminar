# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePoseSubNode(Node):

    def __init__(self):
        super().__init__('turtlepose_sub_node')
        queue_size = 10  # Queue Size
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.sub_callback, queue_size
        )

    def sub_callback(self, msg):
        
        self.get_logger().info(f"""x : {msg.x:.3f} / y : {msg.y:.3f} / z : {msg.theta:.3f}
        linear_velocity : {msg.linear_velocity} / angular_velocity : {msg.angular_velocity }""")


def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = TurtlePoseSubNode()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()