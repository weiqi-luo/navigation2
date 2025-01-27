import os
import subprocess
import sys
import logging
import argparse
from datetime import datetime
from ruamel.yaml import YAML
from dataclasses import dataclass
import re

yaml = YAML()
yaml.indent(sequence=4, offset=2)  # Control indentation


@dataclass
class PathSet:
    current_logs_dir: str
    processing_file: str
    success_file: str
    failed_file: str
    params_file: str
    log_file: str
    bag_file: str


# Function to handle failures and exit the program
def set_flag_file(path_set, flag_file):
    # Clean up old status files in the current logs directory
    for f in [path_set.processing_file, path_set.success_file, path_set.failed_file]:
        if os.path.exists(f):
            os.remove(f)
    with open(flag_file, "w") as f:
        pass
    logging.info(f"Set flag file {flag_file}")


# Function to check if a file or directory exists
def check_file_exists(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"'{path}' does not exist.")


# Function to load YAML content from a file
def read_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File '{file_path}' does not exist.")

    with open(file_path, "r") as file:
        return yaml.load(file)


# Function to save YAML content to a file
def save_yaml(file_path, content):
    with open(file_path, "w") as file:
        yaml.dump(content, file)


# Function to strip ANSI escape codes
def strip_ansi_codes(text):
    ansi_escape = re.compile(r"(?:\x1B[@-_][0-?]*[ -/]*[@-~])")
    return ansi_escape.sub("", text)


# Function to launch ROS2 process with logging and stripping ANSI codes
def launch_ros2(bag_file, params_file, logs_dir, rate, use_rviz):
    try:
        # Run the ros2 launch process and capture output
        process = subprocess.run(
            [
                "ros2",
                "launch",
                "nav2_bringup/launch/iw_nav2_bringup_launch.py",
                f"bag_file:={bag_file}",
                f"params_file:={params_file}",
                f"logs_dir:={logs_dir}",
                f"rate:={rate}",
                f"use_rviz:={use_rviz}",
            ],
            check=True,
            stdout=subprocess.PIPE,  # Capture stdout
            stderr=subprocess.PIPE,  # Capture stderr
            text=True,  # Return output as string
        )

        # Strip ANSI codes and log the output of the command
        stdout_cleaned = strip_ansi_codes(process.stdout)
        stderr_cleaned = strip_ansi_codes(process.stderr)

        logging.info(f"ROS2 launch stdout: {stdout_cleaned}")
        logging.error(f"ROS2 launch stderr: {stderr_cleaned}")

    except subprocess.CalledProcessError as e:
        # Strip ANSI codes from error outputs as well
        stdout_cleaned = strip_ansi_codes(e.stdout)
        stderr_cleaned = strip_ansi_codes(e.stderr)

        logging.error(f"Failed to launch ROS2: {e}")
        logging.error(f"Subprocess stdout: {stdout_cleaned}")
        logging.error(f"Subprocess stderr: {stderr_cleaned}")
        raise e  # Re-raise to handle it higher up


# Function to set up logging
def setup_logging(log_file):
    # Get the root logger
    logger = logging.getLogger()
    # Clear existing handlers to prevent duplication
    if logger.hasHandlers():
        logger.handlers.clear()
    # Set up basic configuration for file logging
    logging.basicConfig(
        filename=log_file,
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    # Set up console logging
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(
        logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    )
    # Add the console handler
    logger.addHandler(console_handler)
    # Log initialization message
    logging.info(f"Logging initialized to {log_file}")


# Function to create logs directories and status files
def create_logs_and_status_files(
    bag_file, logs_dir, param_name, param_value
) -> PathSet:

    # Create logs directory based on the bag_file file and parameter values
    bag_filename = os.path.splitext(os.path.basename(bag_file))[0]
    current_logs_dir = os.path.join(
        logs_dir, f"{bag_filename}/{param_name}/{param_value}"
    )
    os.makedirs(current_logs_dir, exist_ok=True)

    # Status files within the logs directory
    processing_file = os.path.join(current_logs_dir, "processing")
    success_file = os.path.join(current_logs_dir, "succeed")
    failed_file = os.path.join(current_logs_dir, "failed")
    params_file = os.path.join(current_logs_dir, "params.yaml")
    log_file = os.path.join(current_logs_dir, "script_execution.log")

    return PathSet(
        current_logs_dir=current_logs_dir,
        processing_file=processing_file,
        success_file=success_file,
        failed_file=failed_file,
        params_file=params_file,
        log_file=log_file,
        bag_file=bag_file,
    )


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Run the script with bag_file file, params, and YAML inputs."
    )
    parser.add_argument(
        "--bag_file", type=str, required=True, help="Path to the bag_file file"
    )
    parser.add_argument(
        "--params_base",
        type=str,
        required=True,
        help="Path to the original params YAML file",
    )
    parser.add_argument(
        "--params_search",
        type=str,
        required=True,
        help="Path to the YAML file containing parameter names and values",
    )
    parser.add_argument(
        "--logs_dir",
        type=str,
        default="/data/iw/amcl_logs/",
        help="Directory path for logs (default: /data/iw/amcl_logs/)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="rosbag replay rate (default: 1.0)",
    )
    parser.add_argument(
        "--use_rviz",
        type=bool,
        default=False,
        help="if visualize in rviz (default: False)",
    )

    args = parser.parse_args()

    # Read parameters from YAML
    params_dict = read_yaml(args.params_search)
    print(params_dict)

    # Check if the bag_file file and params origin file exist
    check_file_exists(args.bag_file)
    check_file_exists(args.params_base)

    # Iterate over all parameters and values from the YAML file
    for param_name, param_values in params_dict.items():
        for param_value in param_values:
            try:
                path_set = create_logs_and_status_files(
                    args.bag_file, args.logs_dir, param_name, param_value
                )
                setup_logging(path_set.log_file)
                logging.info(
                    f"--------------- {param_name}={param_value} ---------------"
                )
                logging.info(f"bag_file={args.bag_file}")
                set_flag_file(path_set, path_set.processing_file)

                # Load the original param file
                content = read_yaml(args.params_base)

                # Modify param content
                if (
                    "amcl" not in content
                    or "ros__parameters" not in content["amcl"]
                    or param_name not in content["amcl"]["ros__parameters"]
                ):
                    raise KeyError(
                        f"The required structure ['amcl']['ros__parameters'][{param_name}] is missing in the YAML."
                    )
                content["amcl"]["ros__parameters"][param_name] = param_value
                content["amcl"]["ros__parameters"].yaml_add_eol_comment(
                    "Modified by script.", key=param_name
                )

                # Save the modified param file
                save_yaml(path_set.params_file, content)
                logging.info(
                    f"Successfully modified params for {param_name} = {param_value} and saved new params to {path_set.params_file}"
                )

                # Launch ROS2 with the modified parameters and bag_file file
                try:
                    launch_ros2(
                        path_set.bag_file,
                        path_set.params_file,
                        path_set.current_logs_dir,
                        args.rate,
                        args.use_rviz,
                    )
                except Exception as e:
                    logging.error(f"Error launching ROS2: {e}")
                    raise e

            except Exception as e:
                logging.error(f"Fatal error during script execution: {e}")
                set_flag_file(path_set, path_set.failed_file)
                sys.exit(1)

        logging.info("Script execution completed")
        set_flag_file(path_set, path_set.success_file)


if __name__ == "__main__":
    main()
