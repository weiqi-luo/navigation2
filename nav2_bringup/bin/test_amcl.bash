bag=/iw_data/bmw_tests/20240826/logs_v3.0.9/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T114853+0000_2024-08-26-13-08-58_16
bag=/iw_data/bmw_tests/rgb190924/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-09-19T083320+0000_2024-09-19-08-43-26_2
bag=/iw_data/bmw_tests/rgb190924/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-09-19T095044+0000_2024-09-19-13-50-49_48
bag=/iw_data/bmw_tests/rgb190924/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-09-19T083320+0000_2024-09-19-09-13-26_8
bag=/iw_data/bmw_tests/rgb190924/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-09-19T083320+0000_2024-09-19-09-33-26_12

bag=/iw_data/bmw_tests/20240826/logs_v3.0.9/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5
params_file=/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml

# logs_dir
bag_filename=`basename $bag`
bag_filename=${bag_filename%.bag}
current_time=$(date +"%Y%m%d-%H%M%S")
logs_dir="/iw_data/amcl_logs/$bag_filename/$current_time"
mkdir -p $logs_dir

echo bag_filename=$bag_filename
echo current_time=$current_time
echo logs_dir=$logs_dir
echo params_file=$params_file

ros2 launch nav2_bringup/launch/iw_nav2_bringup_launch.py \
    bag_file:=$bag \
    params_file:=$params_file \
    logs_dir:=$logs_dir