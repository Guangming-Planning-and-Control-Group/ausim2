#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include "data/cmd_vel.hpp"

namespace ausim::converts {

void Convert(data::CmdVelData& out, const geometry_msgs::msg::Twist& in);
data::CmdVelData ToCmdVelData(const geometry_msgs::msg::Twist& in);
void Convert(geometry_msgs::msg::Twist& out, const data::CmdVelData& in);
geometry_msgs::msg::Twist ToRosMessage(const data::CmdVelData& in);

}  // namespace ausim::converts
