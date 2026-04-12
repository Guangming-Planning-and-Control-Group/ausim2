#include "converts/data/common.hpp"

#include <cmath>
#include <utility>

namespace ausim::converts {

data::Vector3 ToVector3(const Eigen::Vector3d& value) { return data::Vector3{value.x(), value.y(), value.z()}; }

data::Quaternion ToQuaternion(const Eigen::Quaterniond& value) { return data::Quaternion{value.w(), value.x(), value.y(), value.z()}; }

data::Header BuildHeader(double stamp_seconds, std::string frame_id) {
  data::Header header;
  header.stamp_seconds = stamp_seconds;
  header.frame_id = std::move(frame_id);
  return header;
}

rclcpp::Time ToRosTime(double stamp_seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(stamp_seconds * 1'000'000'000.0)), RCL_ROS_TIME);
}

void Convert(geometry_msgs::msg::Point& out, const data::Vector3& in) {
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

void Convert(geometry_msgs::msg::Vector3& out, const data::Vector3& in) {
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

void Convert(geometry_msgs::msg::Quaternion& out, const data::Quaternion& in) {
  out.w = in.w;
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

}  // namespace ausim::converts
