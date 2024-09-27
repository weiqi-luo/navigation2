# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    # map_yaml_file = LaunchConfiguration("map")
    bag_file = LaunchConfiguration("bag")
    qos_file = LaunchConfiguration("qos", default="/iw_data/config/qos/amcl.yaml")

    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # param_dir = LaunchConfiguration(
    #     "params_file",
    #     default=os.path.join(
    #         bringup_dir,
    #         "params",
    #         "iw_nav2_params.yaml",
    #     ),
    # )

    nav2_launch_file_dir = os.path.join(bringup_dir, "launch")
    rviz_config_baseline = os.path.join("/iw_data/config/rviz/iw_baseline.rviz")
    rviz_config_amcl = os.path.join("/iw_data/config/rviz/iw_amcl.rviz")
    param_dir = os.path.join(
        "/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml"
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "map",
            #     description="Full path to map file to load",
            # ),
            # DeclareLaunchArgument(
            #     "params_file",
            #     default_value=param_dir,
            #     description="Full path to param file to load",
            # ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_launch_file_dir, "/iw_localization_launch.py"]
                ),
                launch_arguments={
                    # "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
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
                    bag_file,
                    "--clock",
                    "--qos-profile-overrides-path",
                    qos_file,
                    "-r",
                    "0.5",
                ],
                output="screen",
            ),
        ]
    )
