import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # rplidar node
    rplidar_node = Node(
        package='rplidar_ros',
        node_executable='rplidarNode',
        node_name='rplidarNode',
        output='screen'
    )

    # Tf publish
    static_transform_publisher = Node(
        package = "tf2_ros", 
        node_executable = "static_transform_publisher",
        arguments = "0 0 0 0 0 0 world laser_frame".split()
    )

    # Rviz with .rviz config
    pkg_path = os.path.join(get_package_share_directory('maro_demo'))
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'lidar_demo.rviz')

    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        TimerAction(
            period=5.0,
            actions=[rviz]
        ),
        rplidar_node,
        static_transform_publisher,
    ])