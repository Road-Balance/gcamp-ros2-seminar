# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class PoseSubTwistPubNode(Node):

    def __init__(self):
        super().__init__('pose_sub_twist_pub_node')
        queue_size = 10  # Queue Size
        self.twist_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", queue_size)
        self.subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.sub_callback, queue_size
        )

    def sub_callback(self, msg):
        pub_msg = Twist()
        pub_msg.linear.x = msg.linear_velocity
        pub_msg.angular.z = msg.angular_velocity

        self.twist_publisher.publish(pub_msg)
        self.get_logger().info(f"""x : {msg.x:.3f} / y : {msg.y:.3f} / z : {msg.theta:.3f}
        linear_velocity : {msg.linear_velocity} / angular_velocity : {msg.angular_velocity }""")


def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = PoseSubTwistPubNode()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()