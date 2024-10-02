import yaml
import argparse


def load_yaml(file_path):
    """
    Load YAML file from the specified file path.

    :param file_path: The path to the YAML file to be loaded.
    :return: A Python dictionary representing the YAML content.
    """
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def save_yaml(file_path, content):
    """
    Save the given content back to the specified YAML file.

    :param file_path: The path to the YAML file to be saved.
    :param content: The content (Python dictionary) to write back to the YAML file.
    """
    with open(file_path, "w") as file:
        yaml.dump(content, file, default_flow_style=False)


def modify_yaml(content):
    """
    Modify the content of a YAML file (loaded as a Python dictionary).

    :param content: The content to be modified.
    :return: The modified content.
    """
    # Example modifications
    content["robot"]["settings"]["speed"] = 2.0  # Modify an existing key
    content["robot"]["settings"]["new_parameter"] = "added_value"  # Add a new key

    return content


def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Modify a YAML file.")
    parser.add_argument(
        "input_yaml",
        help="Path to the input YAML file.",
        type=str,
        default="../params/iw_nav2_params.yaml",
    )
    parser.add_argument(
        "output_yaml",
        help="Path to the output YAML file.",
        type=str,
        default="../tests/iw_nav2_params.yaml",
    )

    # Parse command-line arguments
    args = parser.parse_args()

    # Load the input YAML file
    content = load_yaml(args.input_yaml)

    # Modify the YAML content
    modified_content = modify_yaml(content)

    # Save the modified content to the output YAML file
    save_yaml(args.output_yaml, modified_content)

    print(
        f"YAML file '{args.input_yaml}' has been modified and saved to '{args.output_yaml}'."
    )


if __name__ == "__main__":
    main()
