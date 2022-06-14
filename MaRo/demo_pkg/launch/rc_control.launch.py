
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Rplidar Driver
    rc_control_node = Node(
            package='rccontrol_bringup',
            node_executable='rc_control_node',
            name='rc_control_node',
            output='screen'
        )

    joy_control_node = Node(
            package='rccontrol_bringup',
            node_executable='joy_control_node',
            name='joy_control_node',
            output='screen'
        )

    joy_node = Node(
            package='joy',
            node_executable='joy_node',
            name='joy_node',
            output='screen'
        )

    # Rplidar Driver
    rplidar_ros = Node(
            package='rplidar_ros',
            node_executable='rplidarNode',
            name='rplidarNode',
            output='screen'
        )

    return LaunchDescription([
        rc_control_node,
        joy_control_node,
        joy_node,
        rplidar_ros,
    ])