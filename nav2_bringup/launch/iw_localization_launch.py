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
    # Access launch configurations
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rate = LaunchConfiguration("rate")
    bag_file = LaunchConfiguration("bag_file")
    log_level = LaunchConfiguration("log_level")

    bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_file_dir = os.path.join(bringup_dir, "launch")
    rviz_config_baseline = os.path.join("/data/iw/config/rviz/iw_baseline.rviz")
    rviz_config_amcl = os.path.join("/data/iw/config/rviz/iw_amcl.rviz")
    qos_file = "/data/iw/config/qos/amcl.yaml"

    # Command to play the bag file 
    # Remap /map topic to prevent confusion, since it will be only used for navigation not localization
    # Specify qos profile to handle QoS mismatches after ROS1 to ROS2 bag conversion
    bag_play_cmd = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play",
            bag_file.perform(context),
            "--log-level", "error",
            "--clock",
            "--qos-profile-overrides-path", qos_file,
            "--rate", rate.perform(context),
            "--remap", "/map:=/map_navigation",
        ],
        output="screen",
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, "/iw_amcl_launch.py"]
            ),
            launch_arguments=[('use_sim_time', use_sim_time), ('log_level', log_level)],
        ),
        Node(
            package="nav2_amcl",
            executable="initial_pose_publisher",
            name="initial_pose_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "log_level": log_level}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_baseline",
            arguments=["-d", rviz_config_baseline, "--ros-args", "--log-level", "ERROR"],
            condition=IfCondition(use_rviz),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_amcl",
            arguments=["-d", rviz_config_amcl, "--ros-args", "--log-level", "ERROR"],
            condition=IfCondition(use_rviz),
        ),
        bag_play_cmd,
        # Event handler to shutdown the launch system when the bag file finishes playing
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play_cmd,
                on_exit=[Shutdown()],
            )
        ),
    ]


def generate_launch_description():
    # Declare launch arguments first
    declare_rate_cmd = DeclareLaunchArgument(
        "rate",
        default_value="1.0",
        description="Playback rate for the ROS2 bag file",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for the ROS2 bag file",
    )
    declare_bag_file_cmd = DeclareLaunchArgument(
        "bag_file",
        description="Full path to the ROS2 bag file to play",
    )

    return LaunchDescription(
        [
            # Add declarations first
            declare_rate_cmd,
            declare_use_rviz_cmd,
            declare_use_sim_time_cmd,
            declare_log_level_cmd,
            declare_bag_file_cmd,
            # Then add the setup function
            OpaqueFunction(function=launch_setup),
        ]
    )
