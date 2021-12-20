from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rc_control_node = Node(
        package='rccontrol_bringup',
        node_executable='rc_control_node',
        node_name='rc_control_node',
        output='screen'
    )

    rqt_rc_calibration = Node(
        package='rqt_rc_steering',
        node_executable='rqt_rc_calibration',
        node_name='rqt_rc_calibration',
        output='screen'
    )

    return LaunchDescription([
        rc_control_node,
        rqt_rc_calibration,
    ])
