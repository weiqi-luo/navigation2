import os
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    # Get the launch directory
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    bag_file = LaunchConfiguration("bag_file")
    params_file = LaunchConfiguration("params_file")
    logs_dir = LaunchConfiguration("logs_dir")
    rate = LaunchConfiguration("rate", default="1.0")  # Added rate configuration
    use_remap = LaunchConfiguration("use_remap", default="false")  # New remap flag

    bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_file_dir = os.path.join(bringup_dir, "launch")
    rviz_config_baseline = os.path.join("/data/iw/config/rviz/iw_baseline.rviz")
    rviz_config_amcl = os.path.join("/data/iw/config/rviz/iw_amcl.rviz")
    qos_file = "/data/iw/config/qos/amcl.yaml"

    # Command to play the bag file with rate
    bag_play_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_file.perform(context),
            "--clock",
            "--qos-profile-overrides-path",
            qos_file,
            "--rate",
            rate.perform(context),  # Play the bag file at the specified rate
        ]
        + (
            ["--remap", "/map:=/map_navigation"]
            if use_remap.perform(context) == "true"
            else []
        ),
        output="screen",
    )

    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "params_file",
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
        DeclareLaunchArgument(
            "rate",
            default_value="1.0",
            description="Playback rate for the ROS2 bag file (e.g., 0.5 for half speed, 2.0 for double speed)",
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
            executable="initial_pose_publisher",
            name="initial_pose_publisher",
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
        bag_play_cmd,
        # Event handler to shutdown the launch system when the bag file finishes playing
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play_cmd,
                on_exit=[Shutdown()],  # Correct shutdown action
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
