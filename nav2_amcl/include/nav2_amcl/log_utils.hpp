#ifndef LOG_UTILS_HPP
#define LOG_UTILS_HPP

#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>

namespace nav2_amcl {

namespace fs = std::filesystem;

class LogUtils {
 public:
  static constexpr const char* CSV_HEADER =
      "stamp_sec,stamp_nsec,frame_id,"
      "position_x,position_y,position_z,"
      "orientation_x,orientation_y,orientation_z,orientation_w,"
      "pose_covariance";

  explicit LogUtils(const std::string& output_dir) {
    if (output_dir.empty()) {
      throw std::runtime_error("Output directory cannot be empty");
    }
    if (!fs::exists(output_dir)) {
      fs::create_directories(output_dir);
    }
    output_path_ = (fs::path(output_dir) / "result.csv").string();
    std::ofstream f(output_path_, std::ios::out | std::ios::trunc);
    if (!f) {
      throw std::runtime_error("Failed to create file: " + output_path_);
    }
    f << CSV_HEADER << '\n';
  }

  inline bool logPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    std::ofstream f(output_path_, std::ios::out | std::ios::app);
    if (!f) {
      return false;
    }
    f << pose.header.stamp.sec << ',' << pose.header.stamp.nanosec << ',' << pose.header.frame_id
      << ',' << pose.pose.pose.position.x << ',' << pose.pose.pose.position.y << ','
      << pose.pose.pose.position.z << ',' << pose.pose.pose.orientation.x << ','
      << pose.pose.pose.orientation.y << ',' << pose.pose.pose.orientation.z << ','
      << pose.pose.pose.orientation.w << ',';
    f << "\"[";
    for (const auto& cov : pose.pose.covariance) {
      f << cov << ',';
    }
    f << "]\"\n";
    return f.good();
  }

 private:
  std::string output_path_;
};

}  // namespace nav2_amcl

#endif  // LOG_UTILS_HPP
