
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
    rplidar_ros = Node(
            package='rplidar_ros',
            node_executable='rplidarNode',
            name='rplidarNode',
            output='screen'
        )

    rf2o_laser_odometry = Node(
        package='rf2o_laser_odometry',
        node_executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : True,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
    )

    static_transform_publisher = Node(
        package = "tf2_ros", 
        node_executable = "static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "laser_frame"]
    )

    # Rviz
    slam_toolbox = os.path.join(get_package_share_directory('slam_toolbox'))
    # demo_pkg = os.path.join(get_package_share_directory('demo_pkg'))
    
    slam_params_file = os.path.join(slam_toolbox, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox_with_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        # launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    # rviz_config_dir = os.path.join(demo_pkg, 'rviz', 'rplidar_view.rviz')
    # rviz2 = Node(
    #         package='rviz2',
    #         node_executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', rviz_config_dir],
    #         output='screen'
    #     )

    return LaunchDescription([
        rplidar_ros,
        rf2o_laser_odometry,
        slam_toolbox_with_rviz,
        # rviz2,
    ])