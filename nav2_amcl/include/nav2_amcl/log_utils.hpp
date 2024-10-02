#ifndef LOG_UTILS_HPP
#define LOG_UTILS_HPP

#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nlohmann/json.hpp>  // Include the JSON library
#include <string>

void savePoseToCsv(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose, const std::string& output_path) {
  std::ofstream f;

  // Check if file exists
  bool file_exists = std::ifstream(output_path).good();

  // Open file in append mode
  f.open(output_path, std::ios::out | std::ios::app);
  if (!f.is_open()) {
    throw std::runtime_error("Failed to open file: " + output_path);
  }

  // If file doesn't exist, write the header
  if (!file_exists) {
    f << "stamp_sec,stamp_nsec,frame_id,"
      << "position_x,position_y,position_z,"
      << "orientation_x,orientation_y,orientation_z,orientation_w,"
      << "pose_covariance" << std::endl;
  }

  // Serialize the covariance matrix to a JSON string
  nlohmann::json covariance_json = nlohmann::json::array();
  for (int i = 0; i < 36; ++i) {
    covariance_json.push_back(pose.pose.covariance[i]);
  }
  std::string covariance_str = covariance_json.dump();  // Convert JSON array to string

  // Write data to CSV
  f << pose.header.stamp.sec << "," << pose.header.stamp.nanosec << "," << pose.header.frame_id
    << "," << pose.pose.pose.position.x << "," << pose.pose.pose.position.y << ","
    << pose.pose.pose.position.z << "," << pose.pose.pose.orientation.x << ","
    << pose.pose.pose.orientation.y << "," << pose.pose.pose.orientation.z << ","
    << pose.pose.pose.orientation.w << "," << "\"" << covariance_str << "\"" << std::endl;

  // Close file
  f.close();
}

#endif  // LOG_UTILS_HPP
