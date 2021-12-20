from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rc_control_node = Node(
                        package='rccontrol_bringup',
                        executable='rc_control_node',
                        name='rc_control_node',
                        output='screen'
                        )

    rqt_rc_calibration = Node(
                        package='rqt_rc_steering',
                        executable='rqt_rc_calibration',
                        name='rqt_rc_calibration',
                        output='screen'
                        )

    return LaunchDescription([
        rc_control_node,
        rqt_rc_calibration,
    ])
