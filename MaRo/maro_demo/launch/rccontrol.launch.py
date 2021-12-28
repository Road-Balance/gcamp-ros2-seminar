from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        node_executable='joy_node',
        node_name='joy_node',
        output='screen'
    )

    rc_control_node = Node(
        package='rccontrol_bringup',
        node_executable='rc_control_node',
        node_name='rc_control_node',
        output='screen'
    )

    joy_control_node = Node(
        package='rccontrol_bringup',
        node_executable='joy_control_node',
        node_name='joy_control_node',
        output='screen'
    )


    return LaunchDescription([
        joy_node,
        rc_control_node,
        joy_control_node,
    ])