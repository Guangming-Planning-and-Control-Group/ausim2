#include "converts/data/cmd_vel.hpp"

namespace ausim::converts {

void Convert(data::CmdVelData& out, const geometry_msgs::msg::Twist& in) {
  out.twist.linear.x = in.linear.x;
  out.twist.linear.y = in.linear.y;
  out.twist.linear.z = in.linear.z;
  out.twist.angular.x = in.angular.x;
  out.twist.angular.y = in.angular.y;
  out.twist.angular.z = in.angular.z;
}

data::CmdVelData ToCmdVelData(const geometry_msgs::msg::Twist& in) {
  data::CmdVelData out;
  Convert(out, in);
  return out;
}

void Convert(geometry_msgs::msg::Twist& out, const data::CmdVelData& in) {
  out.linear.x = in.twist.linear.x;
  out.linear.y = in.twist.linear.y;
  out.linear.z = in.twist.linear.z;
  out.angular.x = in.twist.angular.x;
  out.angular.y = in.twist.angular.y;
  out.angular.z = in.twist.angular.z;
}

geometry_msgs::msg::Twist ToRosMessage(const data::CmdVelData& in) {
  geometry_msgs::msg::Twist out;
  Convert(out, in);
  return out;
}

}  // namespace ausim::converts
