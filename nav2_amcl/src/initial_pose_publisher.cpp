#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

// Define constant global variables for frame IDs

// Define how many frames to skip before publishing the initial pose
const int kSkipFrame = 5;

class InitialPosePublisher : public rclcpp::Node {
 public:
  InitialPosePublisher()
      : Node("initial_pose_publisher"),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(*tf_buffer_),
        frame_count_(0) {  // Initialize frame counter

    // Declare the parameter and get its value
    this->declare_parameter<std::string>("map_frame_id_new", "map_amcl");
    this->get_parameter("map_frame_id_new", map_frame_id_new_);

    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(10));

    // Start listening to the transform
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&InitialPosePublisher::listen_to_transform, this));
  }

 private:
  void listen_to_transform() {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Attempt to listen to the transform from 'odom_combined' to 'map'
      transform_stamped =
          tf_buffer_->lookupTransform(map_frame_id_, base_frame_id_, tf2::TimePointZero);

      // Increment frame counter
      frame_count_++;

      // Skip the first kSkipFrame frames
      if (frame_count_ <= kSkipFrame) {
        RCLCPP_INFO(this->get_logger(), "Skipping frame %d/%d", frame_count_, kSkipFrame);
        return;
      }

      // Once we have skipped kSkipFrame frames, convert the transform into a pose and publish it
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.stamp = transform_stamped.header.stamp;
      pose_msg.header.frame_id = map_frame_id_new_;

      // Fill in the Pose information from the transform
      pose_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
      pose_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
      pose_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
      pose_msg.pose.pose.orientation = transform_stamped.transform.rotation;

      // Set a default covariance (for now all 0)
      for (int i = 0; i < 36; ++i) {
        pose_msg.pose.covariance[i] = 0.0;
      }

      // Publish the pose
      initial_pose_pub_->publish(pose_msg);

      RCLCPP_INFO(this->get_logger(), "Published initial pose based on transform from '%s' to '%s'",
          base_frame_id_.c_str(), map_frame_id_new_.c_str());

      // Stop the timer after the first successful transform is received
      timer_->cancel();

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Could not get transform from "
                                                 << map_frame_id_ << " to " << base_frame_id_
                                                 << ": " << ex.what());
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_count_;                               // Frame counter
  std::string map_frame_id_new_;                  // Frame ID that can be set from the parameter
  const std::string map_frame_id_{"map"};         // Frame ID that can be set from the parameter
  const std::string base_frame_id_{"base_link"};  // Frame ID that can be set from the parameter
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
