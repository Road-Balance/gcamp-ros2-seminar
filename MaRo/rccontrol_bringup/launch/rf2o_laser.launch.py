import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # rplidar node
    rplidar_node = Node(
                        package='rplidar_ros',
                        executable='rplidarNode',
                        name='rplidarNode',
                        output='screen'
                        )

    # Tf publish
    static_transform_publisher = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = "0 0 0 0 0 0 world laser_frame".split())

    # Rviz with .rviz config
    pkg_path = os.path.join(get_package_share_directory('sw_ros2_control_gazebo'))
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'gazebo.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rplidar_node,
        static_transform_publisher,
        rviz,
    ])