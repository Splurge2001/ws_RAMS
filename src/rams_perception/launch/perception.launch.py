from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='world',
        description='Target frame for point cloud transformation',
    )

    perception = Node(
        package='rams_perception',
        executable='perception_node',
        name='rams_perception_node',
        output='screen',
        parameters=[{'target_frame': LaunchConfiguration('target_frame')}],
    )

    return LaunchDescription([
        target_frame_arg,
        perception,
    ])
