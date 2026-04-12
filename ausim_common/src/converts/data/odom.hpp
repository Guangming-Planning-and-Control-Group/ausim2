#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <string>

#include "data/odom.hpp"

namespace ausim::converts {

void Convert(nav_msgs::msg::Odometry& out, const data::OdomData& in);
nav_msgs::msg::Odometry ToRosMessage(const data::OdomData& in);

}  // namespace ausim::converts
