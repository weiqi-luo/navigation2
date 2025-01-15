#!/bin/bash

bag_file="/iw_data/bmw_tests/20240826/logs_v3.0.9/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5"
params_base="/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml"
params_search="/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params_search.yaml"

python3 nav2_bringup/script/grid_search.py \
    --bag_file $bag_file \
    --params_base $params_base \
    --params_search $params_search \
    --rate 1 \
    --use_rviz true