import rosbag
import tf2_ros
import rospy
import csv
import argparse
import json
from tf2_msgs.msg import TFMessage


def store_transforms_in_buffer(bag_file):
    rospy.loginfo(f"Opening bag file to store tf in buffer: {bag_file}")
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(999999999))

    # Open the bag file
    bag = rosbag.Bag(bag_file)

    rospy.loginfo("Storing transforms from /tf and /tf_static topics...")
    # Iterate through the /tf and /tf_static topics and store them in the buffer
    for topic, msg, t in bag.read_messages(topics=["/tf", "/tf_static"]):
        assert hasattr(msg, "transforms")
        for transform in msg.transforms:
            if topic == "/tf_static":
                tf_buffer.set_transform_static(transform, "default_authority")
            else:
                tf_buffer.set_transform(transform, "default_authority")

    rospy.loginfo("Finished storing transforms.")
    bag.close()
    return tf_buffer


def query_transform_from_buffer(tf_buffer, target_time, frame_id, child_frame_id):
    try:
        target_time = rospy.Time(target_time)
        transform = tf_buffer.lookup_transform(
            frame_id, child_frame_id, target_time, rospy.Duration(1.0)
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        return {
            "tf_stamp_sec": transform.header.stamp.secs,
            "tf_stamp_nsec": transform.header.stamp.nsecs,
            "tf_position_x": translation.x,
            "tf_position_y": translation.y,
            "tf_position_z": translation.z,
            "tf_orientation_x": rotation.x,
            "tf_orientation_y": rotation.y,
            "tf_orientation_z": rotation.z,
            "tf_orientation_w": rotation.w,
        }

    except tf2_ros.LookupException:
        rospy.logwarn(f"Transform between {frame_id} and {child_frame_id} not found.")
    except tf2_ros.ExtrapolationException:
        rospy.logwarn(
            f"No transform data available around the specified time {target_time.to_sec()}."
        )
    except Exception as e:
        rospy.logerr(f"Error occurred: {e}")

    return None


def store_lts_status_in_list(bag_file):
    lts_status_list = []

    # Open the bag file
    bag = rosbag.Bag(bag_file)

    rospy.loginfo("Storing LTS status messages from /lts_ng/lts_status topic...")
    for topic, msg, t in bag.read_messages(topics=["/lts_ng/lts_status"]):
        # Convert pose covariance and particles_poses_covariance_without_weights to JSON strings
        pose_covariance_json = json.dumps(msg.pose.covariance)
        particles_poses_covariance_without_weights_json = json.dumps(
            msg.particles_poses_covariance_without_weights
        )

        status_entry = {
            "lts_stamp_sec": msg.header.stamp.secs,
            "lts_stamp_nsec": msg.header.stamp.nsecs,
            "lts_loc_confidence": msg.loc_confidence,
            "lts_loc_convergence": msg.loc_convergence,
            "lts_best_particle_weight": msg.best_particle_weight,
            "lts_latest_best_particle_weight": msg.latest_best_particle_weight,
            "lts_static_map_score": msg.static_map_score,
            "lts_long_term_map_score": msg.long_term_map_score,
            "lts_data_match_score": msg.data_match_score,
            "lts_particle_count": msg.particle_count,
            "lts_state": msg.state,
            "lts_position_x": msg.pose.pose.position.x,
            "lts_position_y": msg.pose.pose.position.y,
            "lts_position_z": msg.pose.pose.position.z,
            "lts_orientation_x": msg.pose.pose.orientation.x,
            "lts_orientation_y": msg.pose.pose.orientation.y,
            "lts_orientation_z": msg.pose.pose.orientation.z,
            "lts_orientation_w": msg.pose.pose.orientation.w,
            "lts_pose_covariance": pose_covariance_json,  # Store as JSON string
            "lts_particles_poses_covariance_without_weights": particles_poses_covariance_without_weights_json,  # Store as JSON string
        }
        lts_status_list.append(status_entry)

    bag.close()
    rospy.loginfo(f"Got {len(lts_status_list)} lts status")
    return lts_status_list


def find_closest_lts_status(
    lts_status_list, target_secs, target_nsecs, threshold_secs=1.0
):
    closest_status = None
    closest_time_diff = float("inf")

    target_time_in_ns = target_secs * 1e9 + target_nsecs
    threshold_ns = threshold_secs * 1e9

    for status in lts_status_list:
        status_time_in_ns = status["lts_stamp_sec"] * 1e9 + status["lts_stamp_nsec"]
        time_diff = abs(status_time_in_ns - target_time_in_ns)

        if time_diff < closest_time_diff:
            closest_status = status
            closest_time_diff = time_diff

    if closest_time_diff <= threshold_ns:
        return closest_status
    else:
        rospy.logwarn(
            f"No LTS status found within {threshold_secs} seconds of the target timestamp."
        )
        return None


def process_bag_and_store_to_csv(
    bag_file, topic, frame_id, child_frame_id, output_csv_file
):
    rospy.loginfo(f"Processing bag file: {bag_file}")

    tf_buffer = store_transforms_in_buffer(bag_file)
    lts_status_list = store_lts_status_in_list(bag_file)

    bag = rosbag.Bag(bag_file)

    rospy.loginfo(f"Opening CSV file for writing: {output_csv_file}")
    with open(output_csv_file, mode="w", newline="") as csv_file:
        fieldnames = [
            "timestamp",
            "tf_stamp_sec",
            "tf_stamp_nsec",
            "tf_position_x",
            "tf_position_y",
            "tf_position_z",
            "tf_orientation_x",
            "tf_orientation_y",
            "tf_orientation_z",
            "tf_orientation_w",
            "lts_stamp_sec",
            "lts_stamp_nsec",
            "lts_loc_confidence",
            "lts_loc_convergence",
            "lts_best_particle_weight",
            "lts_latest_best_particle_weight",
            "lts_static_map_score",
            "lts_long_term_map_score",
            "lts_data_match_score",
            "lts_particle_count",
            "lts_state",
            "lts_position_x",
            "lts_position_y",
            "lts_position_z",
            "lts_orientation_x",
            "lts_orientation_y",
            "lts_orientation_z",
            "lts_orientation_w",
            "lts_pose_covariance",
            "lts_particles_poses_covariance_without_weights",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        rospy.loginfo(f"Reading messages from topic: {topic}")
        for topic, msg, t in bag.read_messages(topics=[topic]):
            timestamp = msg.header.stamp.to_sec()
            stamp_sec = msg.header.stamp.secs
            stamp_nsec = msg.header.stamp.nsecs

            transform_data = query_transform_from_buffer(
                tf_buffer, timestamp, frame_id, child_frame_id
            )

            closest_lts_status = find_closest_lts_status(
                lts_status_list, stamp_sec, stamp_nsec
            )

            if transform_data and closest_lts_status:
                output_row = {"timestamp": timestamp}
                output_row.update(transform_data)
                output_row.update(closest_lts_status)
                writer.writerow(output_row)

    rospy.loginfo(f"Data saved to {output_csv_file}")
    bag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Log TF and LTS data from a rosbag to a CSV file."
    )

    parser.add_argument(
        "--bag_file",
        type=str,
        default="../tests/2e08b7de-d742-4e53-b2f0-de131c7bfb48-270-str-2024-08-26T073427+0000_2024-08-26-07-59-33_5.bag",
        help="Path to the input rosbag file",
    )
    parser.add_argument(
        "--csv_file",
        type=str,
        default="../tests/baseline.csv",
        help="Path to the output CSV file",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/scan_unified",
        help="Topic name to read messages from",
    )
    parser.add_argument(
        "--frame_id",
        type=str,
        default="map",
        help="Frame ID to filter by",
    )
    parser.add_argument(
        "--child_frame_id",
        type=str,
        default="base_link",
        help="Child frame ID to filter by",
    )

    args = parser.parse_args()

    rospy.loginfo("Initializing ROS node...")
    rospy.init_node("tf_query_node", anonymous=True, disable_signals=True)

    process_bag_and_store_to_csv(
        args.bag_file, args.topic, args.frame_id, args.child_frame_id, args.csv_file
    )
