params_file=/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml
# map_yaml_file="/data/iw/bmw_tests/20241002/899_no_initial_pose_0716/map_0.2/20241001T045712Z.yaml"
map_yaml_file="/data/iw/bmw_tests/20240826/node_env/2e08b7de-d742-4e53-b2f0-de131c7bfb48/map/map_0.2/20240826T125010Z.yaml"
map_topic_name="map_amcl"
echo params_file=$params_file
echo map_yaml_file=$map_yaml_file
echo map_topic_name=$map_topic_name
# Check if params_file exists
if [ ! -f "$params_file" ]; then
  echo "Error: params_file does not exist."
  exit 1
fi

# Check if map_yaml_file exists
if [ ! -f "$map_yaml_file" ]; then
  echo "Error: map_yaml_file does not exist."
  exit 1
fi

ros2 launch nav2_bringup/launch/iw_map_server_launch.py \
    params_file:=$params_file \
    map_yaml_file:=$map_yaml_file \
    map_topic_name:=$map_topic_name
  
  