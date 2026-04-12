#include "converts/data/odom.hpp"

#include "converts/data/common.hpp"

namespace ausim::converts {

void Convert(nav_msgs::msg::Odometry& out, const data::OdomData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  out.child_frame_id = in.child_frame_id;
  Convert(out.pose.pose.position, in.pose.position);
  Convert(out.pose.pose.orientation, in.pose.orientation);
  Convert(out.twist.twist.linear, in.twist.linear);
  Convert(out.twist.twist.angular, in.twist.angular);
}

nav_msgs::msg::Odometry ToRosMessage(const data::OdomData& in) {
  nav_msgs::msg::Odometry out;
  Convert(out, in);
  return out;
}

}  // namespace ausim::converts
