from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ---- Launch 参数（可按需覆盖）----
    fixed_serial = DeclareLaunchArgument(
        "fixed_serial", default_value="109522062115",  # 注意：去掉了你命令里的下划线 "_"
        description="Serial number of the fixed RealSense camera"
    )
    wrist_serial = DeclareLaunchArgument(
        "wrist_serial", default_value="342522070232",  # 注意：去掉了你命令里的下划线 "_"
        description="Serial number of the wrist RealSense camera"
    )

    # 包路径
    realsense_share = get_package_share_directory("realsense2_camera")
    abb_bringup_share = get_package_share_directory("abb_bringup")
    workcell_description_share = get_package_share_directory("workcell_description")
    workcell_config_share = get_package_share_directory("workcell_config")

    # ========== 相机 ==========
    # fixed_cam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(realsense_share, "launch", "rs_launch.py")
    #     ),
    #     launch_arguments={
    #         "camera_name": "fixed_cam",
    #         "camera_namespace": "/fixed_cam",
    #         "serial_no": LaunchConfiguration("fixed_serial"),
    #         "align_depth": "true",
    #         "pointcloud.enable": "true",
    #         "color_qos": "SENSOR_DATA",
    #         "depth_qos": "SENSOR_DATA",
    #     }.items(),
    # )

    wrist_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "wrist_cam",
            "camera_namespace": "/wrist_cam",
            "serial_no": LaunchConfiguration("wrist_serial"),
            "align_depth": "true",
            "pointcloud.enable": "true",
            "color_qos": "SENSOR_DATA",
            "depth_qos": "SENSOR_DATA",
        }.items(),
    )

    # ========== 外参（静态 TF）==========
    # base_link -> fixed_cam_link
    tf_fixed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_fixed_cam",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "fixed_cam_link"],
        output="screen",
    )
    # tool0 -> wrist_cam_link
    tf_wrist = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_wrist_cam",
        arguments=["0", "0", "0", "0", "0", "0", "tool0", "wrist_cam_link"],
        output="screen",
    )

    # ========== ABB 控制 & MoveIt ==========
    abb_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_bringup_share, "launch", "abb_control.launch.py")
        ),
        launch_arguments={
            "description_package": "workcell_description",
            "description_file": "workcell.urdf.xacro",
            "use_fake_hardware": "false",
            "rws_ip": "192.168.146.1",
            "egm_port": "6512",
            "launch_rviz": "false",
        }.items(),
    )

    abb_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(abb_bringup_share, "launch", "abb_moveit.launch.py")
        ),
        launch_arguments={
            "robot_xacro_file": "workcell.urdf.xacro",
            "support_package": "workcell_description",
            "moveit_config_package": "workcell_config",
            "moveit_config_file": "workcell.srdf",
            "use_fake_hardware": "false",
            "launch_rviz": "true",
        }.items(),
    )

    # ========== 你的节点 ==========
    perception = Node(
        package="rams_perception",
        executable="perception_node",
        name="perception_node",
        output="screen",
    )

    move_to_pose_server = Node(
        package="rams_interface",
        executable="move_to_pose_server",
        name="move_to_pose_server",
        output="screen",
        # 如果你的服务端支持参数 `planning_group`，可在此指定：
        # parameters=[{"planning_group": "manipulator"}],
    )

    return LaunchDescription([
        fixed_serial, wrist_serial,
        fixed_cam, wrist_cam,
        tf_fixed, tf_wrist,
        abb_control, abb_moveit,
        perception, move_to_pose_server,
    ])
