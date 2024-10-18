import argparse
import logging
import os
from ruamel.yaml import YAML

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

yaml = YAML()
yaml.indent(sequence=4, offset=2)  # Control indentation


def load_yaml(file_path):
    """
    Load a YAML file from the specified file path.

    :param file_path: The path to the YAML file to be loaded.
    :return: A Python dictionary representing the YAML content.
    :raises FileNotFoundError: If the file at the given path doesn't exist.
    :raises yaml.YAMLError: If the file contains invalid YAML.
    """
    if not os.path.exists(file_path):
        logging.error(f"File '{file_path}' does not exist.")
        raise FileNotFoundError(f"The file '{file_path}' was not found.")

    try:
        with open(file_path, "r") as file:
            return yaml.load(file)
    except Exception as exc:
        logging.error(f"Error while parsing YAML file '{file_path}': {exc}")
        raise


def save_yaml(file_path, content):
    """
    Save the given content back to the specified YAML file.

    :param file_path: The path to the YAML file to be saved.
    :param content: The content (Python dictionary) to write back to the YAML file.
    :raises IOError: If there is an issue writing to the file.
    """
    try:
        with open(file_path, "w") as file:
            yaml.dump(content, file)
        logging.info(f"YAML content successfully saved to '{file_path}'.")
    except IOError as exc:
        logging.error(f"Failed to save YAML file '{file_path}': {exc}")
        raise


def set_param(params, param_name, param_value):
    """
    Update the parameter in the dictionary with the provided value and add a comment.

    :param params: The dictionary containing the YAML parameters.
    :param param_name: The parameter name to modify.
    :param param_value: The new value for the specified parameter.
    :return: The modified content or None if the parameter doesn't exist.
    """
    if param_name in params:
        params[param_name] = param_value

        # Add comment to the parameter using ruamel.yaml's CommentedMap
        params.yaml_add_eol_comment("Modified by script.", key=param_name)

        logging.info(
            f"Parameter '{param_name}' updated to '{param_value}' with comment."
        )
        return params
    else:
        logging.warning(
            f"Parameter '{param_name}' not found in the provided YAML content."
        )
        return None


def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Modify a YAML file.")
    parser.add_argument(
        "--input_yaml",
        help="Path to the input YAML file.",
        type=str,
        default="/home/user/ros2_ws/src/navigation2/nav2_bringup/params/iw_nav2_params.yaml",
    )
    parser.add_argument(
        "--output_yaml",
        help="Path to the output YAML file.",
        type=str,
        default="/home/user/ros2_ws/src/navigation2/nav2_bringup/tests/iw_nav2_params.yaml",
    )
    parser.add_argument(
        "--param_name",
        help="The parameter name in the YAML file to modify.",
        type=str,
        required=True,
    )
    parser.add_argument(
        "--param_value",
        help="The new value for the specified parameter.",
        type=str,  # We keep this as a string and handle conversions later
        required=True,
    )

    # Parse command-line arguments
    args = parser.parse_args()

    # Validate input parameters
    if not args.param_name:
        logging.error("Parameter name is required.")
        return

    # Load the input YAML file
    try:
        content = load_yaml(args.input_yaml)
    except (FileNotFoundError, Exception) as e:
        logging.error(f"Failed to load YAML file: {e}")
        return

    # Try to convert param_value to float if possible, otherwise treat as string
    try:
        param_value = float(args.param_value)
        logging.info(f"Parameter value '{args.param_value}' parsed as float.")
    except ValueError:
        param_value = args.param_value
        logging.info(f"Parameter value '{args.param_value}' treated as string.")

    # Modify the YAML content based on the passed parameters
    try:
        modified_content = set_param(
            content["amcl"]["ros__parameters"], args.param_name, param_value
        )
        if modified_content:
            content["amcl"]["ros__parameters"] = modified_content
            # Save the modified content to the output YAML file
            save_yaml(args.output_yaml, content)
            logging.info(
                f"YAML file '{args.input_yaml}' has been modified and saved to '{args.output_yaml}'."
            )
        else:
            logging.error("Parameter modification failed, parameter not found.")
    except KeyError as e:
        logging.error(f"Error: '{e}' key not found in the YAML structure.")


if __name__ == "__main__":
    main()
