cd ../..
if [ $# -eq 0 ]; then
    ./scripts/ci/build_with_tests.sh --packages-select \
    nav2_common \
    nav2_msgs \
    nav2_util \
    nav2_lifecycle_manager \
    nav2_map_server \
    nav2_rviz_plugins \
    nav2_amcl \
    navigation2 \
    nav2_bringup \
    cartographer_ros
else
    ./scripts/ci/build_with_tests.sh --packages-select $@
fi
cd -