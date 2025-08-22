from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rams_interface',
            executable='gui_node',
            name='rams_gui',
            output='screen'
        )
    ])