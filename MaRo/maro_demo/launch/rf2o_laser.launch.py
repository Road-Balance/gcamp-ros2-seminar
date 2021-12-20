import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # rplidar node
    rplidar_node = Node(
        package='rplidar_ros',
        node_executable='rplidarNode',
        node_name='rplidarNode',
        output='screen'
    )

    # rf2o node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        node_executable='rf2o_laser_odometry_node',
        node_name='rf2o_laser_odometry',
        output='screen',
        parameters=[
            {
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom_rf2o',
                'publish_tf' : True,
                'base_frame_id' : 'base_footprint',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 10.0
            }
        ],
    )

    # Tf publish
    static_transform_publisher = Node(
        package = "tf2_ros", 
        node_executable = "static_transform_publisher",
        arguments = "0 0 0 0 0 0 world laser_frame".split()
    )

    # Rviz with .rviz config
    pkg_path = os.path.join(get_package_share_directory('maro_demo'))
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'gazebo.rviz')

    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rplidar_node,
        rf2o_node,
        static_transform_publisher,
        # rviz,
    ])