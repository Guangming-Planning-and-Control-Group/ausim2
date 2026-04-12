#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>

#include "data/transform.hpp"

namespace ausim::converts {

void Convert(geometry_msgs::msg::TransformStamped& out, const data::TransformData& in);
geometry_msgs::msg::TransformStamped ToRosMessage(const data::TransformData& in);

}  // namespace ausim::converts
