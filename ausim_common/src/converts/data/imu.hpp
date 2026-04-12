#pragma once

#include <string>
#include <sensor_msgs/msg/imu.hpp>

#include "data/imu.hpp"

namespace ausim::converts {

void Convert(sensor_msgs::msg::Imu& out, const data::ImuData& in);
sensor_msgs::msg::Imu ToRosMessage(const data::ImuData& in);

}  // namespace ausim::converts
