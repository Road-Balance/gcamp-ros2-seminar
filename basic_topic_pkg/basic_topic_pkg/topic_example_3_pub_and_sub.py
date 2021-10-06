# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# Copyright 2021 @RoadBalance
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
This is an third example code for ROS 2 rclpy node programming.

Let's learn about those things.

Implement Example 3 with ROS 2 Node Composition.
"""
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