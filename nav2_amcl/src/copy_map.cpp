#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map_amcl", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

 private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const {
    RCLCPP_INFO(
        this->get_logger(), "map received with frame id: '%s'", msg->header.frame_id.c_str());
    nav_msgs::msg::OccupancyGrid msg_copy = *msg;
    msg_copy.header.frame_id = "map_amcl";
    occ_pub_->publish(msg_copy);
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}