#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class InitialPosePublisher : public rclcpp::Node {
 public:
  InitialPosePublisher()
      : Node("initial_pose_publisher"),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(*tf_buffer_) {
    // Declare the parameter and get its value
    this->declare_parameter<std::string>("map_frame_id_new", "map_amcl");
    this->get_parameter("map_frame_id_new", map_frame_id_new_);
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->get_parameter("map_frame_id", map_frame_id_);
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->get_parameter("base_frame_id", base_frame_id_);

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

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Could not get transform from %s to %s: %s", map_frame_id_.c_str(),
          base_frame_id_.c_str(), ex.what());
      return;
    }
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

    RCLCPP_INFO(this->get_logger(),
        "Got transform from '%s' to '%s' and published transform from '%s' to '%s'",
        base_frame_id_.c_str(), map_frame_id_.c_str(), map_frame_id_.c_str(),
        map_frame_id_new_.c_str());

    // Stop the timer after the first successful transform is received
    timer_->cancel();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string map_frame_id_new_;                  // Frame ID that can be set from the parameter
  std::string map_frame_id_{"map"};               // Frame ID that can be set from the parameter
  std::string base_frame_id_{"base_link"};        // Frame ID that can be set from the parameter
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
