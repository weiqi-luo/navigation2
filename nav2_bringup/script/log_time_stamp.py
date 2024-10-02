#!/usr/bin/env python

import rosbag
import csv
import rospy
import argparse
import pandas as pd


def log_timestamps(bag_file, csv_file):
    # Initialize ROS node
    rospy.init_node("log_timestamps", anonymous=False)

    # Open the rosbag file
    data = []
    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages():
            # Extract header timestamp
            if topic in [
                "/obstacle_collision_filter/obstacle_zone",
                "/bmwstr_zone_model/zone_manager_node/visualization",
                "/interaction_manager/repause_monitor/status",
                "/isaac/debug_ts",
                "/laser_front/scan_planning",
                "/laser_rear/scan_planning",
            ]:
                continue
            elif hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                stamp_sec = msg.header.stamp.secs
                stamp_nsec = msg.header.stamp.nsecs
                length = 1
            elif hasattr(msg, "transforms"):
                length = len(msg.transforms)
                stamp_sec = msg.transforms[0].header.stamp.secs if length > 0 else None
                stamp_nsec = (
                    msg.transforms[0].header.stamp.nsecs if length > 0 else None
                )
            elif hasattr(msg, "markers"):
                length = len(msg.markers)
                stamp_sec = (
                    msg.markers[1].header.stamp.secs
                    if length > 1
                    else msg.markers[0].header.stamp.secs
                )
                stamp_nsec = (
                    msg.markers[1].header.stamp.nsecs
                    if length > 1
                    else msg.markers[0].header.stamp.nsecs
                )
                for iter in msg.markers[1:]:
                    assert iter.header.stamp.secs == stamp_sec
                    assert iter.header.stamp.nsecs == stamp_nsec
            elif type(msg)._type in [
                "actionlib_msgs/GoalID",
                "std_msgs/String",
                "std_msgs/Float32",
                "std_msgs/Bool",
                "geometry_msgs/Twist",
                "ipa_navigation_msgs/NavigationZoneList",
            ]:
                continue
            else:
                print("cannot handle topic [", topic, "] in type", type(msg)._type)
                continue

            # Append the data to a list
            data.append([t.to_sec(), stamp_sec, stamp_nsec, topic, length])

    # Convert the data to a Pandas DataFrame
    df = pd.DataFrame(
        data, columns=["timestamp", "stamp_sec", "stamp_nsec", "topic_name", "length"]
    )

    # Sort by 'stamp_sec' and 'stamp_nsec'
    df_sorted = df.sort_values(by=["stamp_sec", "stamp_nsec"], na_position="last")

    # Save the sorted DataFrame to a CSV file
    df_sorted.to_csv(csv_file, index=False)

    rospy.loginfo("Timestamps logged and sorted to CSV file: %s", csv_file)


if __name__ == "__main__":
    # Argument parser for command-line arguments
    parser = argparse.ArgumentParser(
        description="Log timestamps and topic names from a rosbag to a CSV file."
    )
    parser.add_argument(
        "--bag_file",
        type=str,
        default="../tests/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5.bag",
        help="Path to the input rosbag file (default: ../tests/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5.bag)",
    )
    parser.add_argument(
        "--csv_file",
        type=str,
        default="../tests/timestamps_log.csv",
        help="Path to the output CSV file (default: timestamps_log.csv)",
    )

    args = parser.parse_args()

    log_timestamps(args.bag_file, args.csv_file)
