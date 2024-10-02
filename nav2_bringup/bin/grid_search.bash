#!/bin/bash
# bag
bag=/iw_data/bmw_tests/20240826/logs_v3.0.9/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5

# log_dir
bag_filename="${bag_file%.bag}"
current_time=$(date +"%Y%m%d_%H%M%S")
log_dir="/iw_data/amcl_logs/$bag_filename/$current_time"
mkdir -p "$log_dir"

params_origin=/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml
params=
ros2 launch nav2_bringup/launch/iw_nav2_bringup_launch.py \
    bag:=$bag