from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='manipulator',
        description='MoveIt planning group'
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('workcell_config'),
                'launch',
                'move_group.launch.py'
            ])
        )
    )

    move_to_pose_node = Node(
        package='rams_interface',
        executable='move_to_pose_server',
        name='move_to_pose_server',
        output='screen',
        parameters=[{'planning_group': LaunchConfiguration('planning_group')}]
    )

    return LaunchDescription([
        planning_group_arg,
        move_group_launch,
        move_to_pose_node
    ])