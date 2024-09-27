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
const std::string kOdomFrameIdOld = "map";
const std::string kOdomFrameIdNew = "map_amcl";
const std::string kBaseFrameId = "base_link";

// Define how many frames to skip before publishing the initial pose
const int kSkipFrame = 5;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber()
      : Node("minimal_subscriber"),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(*tf_buffer_),
        frame_count_(0) {  // Initialize frame counter

    subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(kOdomFrameIdOld,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        kOdomFrameIdNew, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(10));

    // Start listening to the transform
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&MinimalSubscriber::listen_to_transform, this));
  }

 private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const {
    RCLCPP_INFO(
        this->get_logger(), "map received with frame id: '%s'", msg->header.frame_id.c_str());
    nav_msgs::msg::OccupancyGrid msg_copy = *msg;
    msg_copy.header.frame_id = kOdomFrameIdNew;
    occ_pub_->publish(msg_copy);
  }

  void listen_to_transform() {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Attempt to listen to the transform from 'odom_combined' to 'map'
      transform_stamped = tf_buffer_->lookupTransform(
          kOdomFrameIdOld, kBaseFrameId, tf2::TimePointZero);

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
      pose_msg.header.frame_id = kOdomFrameIdNew;

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
          kBaseFrameId.c_str(), kOdomFrameIdNew.c_str());

      // Stop the timer after the first successful transform is received
      timer_->cancel();

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_count_;  // Frame counter
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
