#pragma once

#include <rosgraph_msgs/msg/clock.hpp>

#include "data/clock.hpp"

namespace ausim::converts {

data::ClockData ToClockData(double stamp_seconds);

void Convert(rosgraph_msgs::msg::Clock& out, const data::ClockData& in);
rosgraph_msgs::msg::Clock ToRosMessage(const data::ClockData& in);

}  // namespace ausim::converts
