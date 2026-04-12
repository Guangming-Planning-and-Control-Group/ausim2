#pragma once

#include <sensor_msgs/msg/image.hpp>

#include "data/image.hpp"

namespace ausim::converts {

void Convert(sensor_msgs::msg::Image& out, const data::ImageData& in);
sensor_msgs::msg::Image ToRosMessage(const data::ImageData& in);

}  // namespace ausim::converts
