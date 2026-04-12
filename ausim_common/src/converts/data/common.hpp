#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>

#include "data/common.hpp"

namespace ausim::converts {

data::Vector3 ToVector3(const Eigen::Vector3d& value);
data::Quaternion ToQuaternion(const Eigen::Quaterniond& value);
data::Header BuildHeader(double stamp_seconds, std::string frame_id);

rclcpp::Time ToRosTime(double stamp_seconds);
void Convert(geometry_msgs::msg::Point& out, const data::Vector3& in);
void Convert(geometry_msgs::msg::Vector3& out, const data::Vector3& in);
void Convert(geometry_msgs::msg::Quaternion& out, const data::Quaternion& in);

}  // namespace ausim::converts
