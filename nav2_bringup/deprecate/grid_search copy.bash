#!/bin/bash

# Set the bag file path
bag="/iw_data/bmw_tests/20240826/logs_v3.0.9/bags_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5"

# Set the parameter name and values
param_name="alpha1"
param_values=(1 2 3)

# Enable logging
log_file="/iw_data/amcl_logs/script_execution.log"
exec > >(tee -i "$log_file") 2>&1

# Handle Ctrl+C (SIGINT)
trap ctrl_c INT

function ctrl_c() {
    echo "Script interrupted by user (Ctrl+C). Stopping the script at $(date)." | tee -a "$log_file"
    touch "$current_logs_dir/failed"  # Create failed file on interruption
    exit 1
}

# Function to check if a file or directory exists
function check_file_exists() {
    if [ ! -f "$1" ] && [ ! -d "$1" ]; then
        echo "Error: '$1' does not exist. Exiting..." | tee -a "$log_file"
        touch "$current_logs_dir/failed"  # Create failed file if necessary files don't exist
        exit 1
    fi
}

# Log the start of the script
echo "Starting script execution at $(date)"
echo "Bag file: $bag"
echo "Param name: $param_name"
echo "Param values: ${param_values[*]}"

# Check if the bag file exists
check_file_exists "$bag"

# Check if the params origin file exists
params_origin="/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml"
check_file_exists "$params_origin"

# Loop through the param_values
for param_value in "${param_values[@]}"; do
    echo "------------------------------------------"
    echo "Processing param_value: $param_value at $(date)"

    # Extract filename from bag file without the extension
    bag_filename=$(basename "${bag}" .bag)

    # Create logs directory based on the bag file and parameter values
    current_logs_dir="/iw_data/amcl_logs/${bag_filename}/${param_name}/${param_value}"
    echo "Creating logs directory: $current_logs_dir"
    mkdir -p "$current_logs_dir"

    # Status files within the logs directory
    processing_file="$current_logs_dir/processing"
    success_file="$current_logs_dir/succeed"
    failed_file="$current_logs_dir/failed"

    # Clean up old status files in the current logs directory
    rm -f "$processing_file" "$success_file" "$failed_file"

    # Create processing status file
    touch "$processing_file"

    # Set the paths for the params files
    params_file="${current_logs_dir}/params.yaml"
    echo "Params origin: $params_origin"
    echo "Params file (output): $params_file"

    # Modify the YAML parameters using the Python script
    echo "Modifying parameters for param_value: $param_value"
    python3 nav2_bringup/script/modify_params.py \
        --input_yaml "$params_origin" \
        --output_yaml "$params_file" \
        --param_name "$param_name" \
        --param_value "$param_value"

    # Check if the Python script ran successfully
    if [ $? -ne 0 ]; then
        echo "Error: Failed to modify params for value $param_value at $(date)."
        touch "$failed_file"  # Create failed status file
        rm -f "$processing_file"
        exit 1
    else
        echo "Successfully modified params for value $param_value."
    fi

    # Launch ROS2 with the modified parameters and bag file
    echo "Launching ROS2 with bag file and params for param_value: $param_value"
    ros2 launch nav2_bringup/launch/iw_nav2_bringup_launch.py \
        bag_file:="$bag" \
        params_file:="$params_file" \
        logs_dir:="$current_logs_dir" \
        use_rviz:=false \
        rate:=1

    # Check if the launch was successful
    if [ $? -ne 0 ]; then
        echo "Error: Failed to launch ROS2 for param_value $param_value at $(date)."
        touch "$failed_file"  # Create failed status file
        rm -f "$processing_file"
        exit 1
    else
        echo "Successfully launched ROS2 for param_value $param_value at $(date)."
    fi

    echo "Finished processing param_value: $param_value at $(date)"
    echo "------------------------------------------"

    # Clean up processing status file and create success status file
    rm -f "$processing_file"
    touch "$success_file"
done

# Log the end of the script
echo "Script execution completed at $(date)"
