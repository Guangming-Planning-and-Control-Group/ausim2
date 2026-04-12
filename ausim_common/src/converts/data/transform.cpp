#include "converts/data/transform.hpp"

#include "converts/data/common.hpp"

namespace ausim::converts {

void Convert(geometry_msgs::msg::TransformStamped& out, const data::TransformData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  out.child_frame_id = in.child_frame_id;
  Convert(out.transform.translation, in.translation);
  Convert(out.transform.rotation, in.rotation);
}

geometry_msgs::msg::TransformStamped ToRosMessage(const data::TransformData& in) {
  geometry_msgs::msg::TransformStamped out;
  Convert(out, in);
  return out;
}

}  // namespace ausim::converts
