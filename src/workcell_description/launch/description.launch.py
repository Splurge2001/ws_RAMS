from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('workcell_description')
    xacro = os.path.join(pkg, 'urdf', 'workcell.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': False,
                         'robot_description': open(xacro).read()}],
            output='screen'
        ),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui')
    ])
