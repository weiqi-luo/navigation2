#!/usr/bin/env python

import rosbag
import csv
import rospy
import argparse
from tf.msg import tfMessage


def log_tf_to_csv(bag_file, csv_file, frame_id_filter):
    # Initialize ROS node
    rospy.init_node("log_tf_to_csv", anonymous=True)

    # Open the rosbag file
    with rosbag.Bag(bag_file, "r") as bag:
        # Open the CSV file for writing
        with open(csv_file, mode="w", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)
            # Write the header with the specified order
            csv_writer.writerow(
                [
                    "timestamp",
                    "header_stamp_sec",
                    "header_stamp_nsec",
                    "frame_id",
                    "child_frame_id",
                    "translation_x",
                    "translation_y",
                    "translation_z",
                    "rotation_x",
                    "rotation_y",
                    "rotation_z",
                    "rotation_w",
                ]
            )

            for topic, msg, t in bag.read_messages(topics=["/tf"]):
                # Iterate through all transformations in the message
                for transform in msg.transforms:
                    # Extract the transformation data
                    frame_id = transform.header.frame_id
                    child_frame_id = transform.child_frame_id

                    # Log only if frame_id matches the filter
                    if frame_id == frame_id_filter or child_frame_id == frame_id_filter:
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        header_stamp_sec = (
                            transform.header.stamp.secs
                        )  # Get header stamp seconds
                        header_stamp_nsec = (
                            transform.header.stamp.nsecs
                        )  # Get header stamp nanoseconds

                        # Write the data to the CSV file
                        csv_writer.writerow(
                            [
                                t.to_sec(),  # Timestamp of the message
                                header_stamp_sec,  # Header stamp seconds
                                header_stamp_nsec,  # Header stamp nanoseconds
                                frame_id,
                                child_frame_id,
                                translation.x,
                                translation.y,
                                translation.z,
                                rotation.x,
                                rotation.y,
                                rotation.z,
                                rotation.w,
                            ]
                        )

    rospy.loginfo("TF data logged to CSV file: %s", csv_file)


if __name__ == "__main__":
    # Argument parser for command-line arguments
    parser = argparse.ArgumentParser(
        description="Log TF data from a rosbag to a CSV file."
    )
    parser.add_argument(
        "--bag_file",
        type=str,
        default="../tests/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5.bag",
        help="Path to the input rosbag file (default: /workspace/iw_ws/iw_hub_ros2/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5.bag)",
    )
    parser.add_argument(
        "--csv_file",
        type=str,
        default="../tests/tf.csv",
        help="Path to the output CSV file (default: /workspace/iw_ws/iw_hub_ros2/tf.csv)",
    )
    parser.add_argument(
        "--frame_id",
        type=str,
        default="odom_combined",
        help="Frame ID to filter by (default: map)",
    )

    args = parser.parse_args()

    # Log the initial parameters being used
    rospy.loginfo(
        "Using parameters:\n Bag File: %s\n CSV File: %s\n Frame ID: %s",
        args.bag_file,
        args.csv_file,
        args.frame_id,
    )

    log_tf_to_csv(args.bag_file, args.csv_file, args.frame_id)
