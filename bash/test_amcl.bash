bag=/data/iw/bmw_tests/20240826/logs_v3.0.9/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5
# bag=/data/iw/bmw_tests/20241002/899_no_initial_pose_0716/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-09-30T065541+0000_2024-10-01-05-10-47_267
params_file=/home/user/ros2_ws/src/navigation2/nav2_bringup/tests/iw_nav2_params.yaml

# logs_dir
bag_filename=`basename $bag`
bag_filename=${bag_filename%.bag}
current_time=$(date +"%Y%m%d-%H%M%S")
logs_dir="/data/iw/amcl_logs/$bag_filename/$current_time"
mkdir -p $logs_dir

echo params_file=$params_file
echo logs_dir=$logs_dir
echo bag_filename=$bag_filename
echo current_time=$current_time


# Check if params_file exists
if [ ! -f "$params_file" ]; then
  echo "Error: params_file does not exist."
  exit 1
fi

# Check if logs_dir exists
if [ ! -d "$logs_dir" ]; then
  echo "Error: logs_dir does not exist."
  exit 1
fi

ros2 launch nav2_bringup/launch/iw_nav2_bringup_launch.py \
    bag_file:=$bag \
    params_file:=$params_file \
    logs_dir:=$logs_dir \
    use_remap:=true
