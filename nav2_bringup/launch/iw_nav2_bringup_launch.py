import os
from datetime import datetime
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    # Get the launch directory
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    bag_file = LaunchConfiguration("bag_file")
    params_file = LaunchConfiguration("params_file")
    logs_dir = LaunchConfiguration("logs_dir")

    bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_file_dir = os.path.join(bringup_dir, "launch")
    rviz_config_baseline = os.path.join("/iw_data/config/rviz/iw_baseline.rviz")
    rviz_config_amcl = os.path.join("/iw_data/config/rviz/iw_amcl.rviz")
    qos_file = "/iw_data/config/qos/amcl.yaml"

    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "params_file",
            # default_value="/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml",
            description="Full path to the parameters file to use for the Nav2 launch",
        ),
        DeclareLaunchArgument(
            "bag_file",
            description="Full path to the ROS2 bag file to play",
        ),
        DeclareLaunchArgument(
            "logs_dir",
            description="Directory to store logs",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, "/iw_localization_launch.py"]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "logs_dir": logs_dir,
                "log_level": "info",
            }.items(),
        ),
        Node(
            package="nav2_amcl",
            executable="copy_map",
            name="copy_map",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_baseline",
            arguments=["-d", rviz_config_baseline],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_amcl",
            arguments=["-d", rviz_config_amcl],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                bag_file.perform(context),
                "--clock",
                "--qos-profile-overrides-path",
                qos_file,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
