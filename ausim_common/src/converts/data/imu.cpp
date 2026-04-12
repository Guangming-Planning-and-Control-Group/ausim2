#include "converts/data/imu.hpp"

#include "converts/data/common.hpp"

namespace ausim::converts {

void Convert(sensor_msgs::msg::Imu& out, const data::ImuData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  Convert(out.orientation, in.orientation);
  Convert(out.angular_velocity, in.angular_velocity);
  if (in.has_linear_acceleration) {
    Convert(out.linear_acceleration, in.linear_acceleration);
  }
}

sensor_msgs::msg::Imu ToRosMessage(const data::ImuData& in) {
  sensor_msgs::msg::Imu out;
  Convert(out, in);
  return out;
}

}  // namespace ausim::converts
