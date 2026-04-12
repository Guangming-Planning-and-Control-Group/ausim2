#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <string>

#include "data/imu.hpp"

namespace ausim::converts {

void Convert(sensor_msgs::msg::Imu& out, const data::ImuData& in);
sensor_msgs::msg::Imu ToRosMessage(const data::ImuData& in);

}  // namespace ausim::converts
