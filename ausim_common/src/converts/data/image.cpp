#include "converts/data/image.hpp"

#include "converts/data/common.hpp"

namespace ausim::converts {

void Convert(sensor_msgs::msg::Image& out, const data::ImageData& in) {
  out.header.stamp = ToRosTime(in.header.stamp_seconds);
  out.header.frame_id = in.header.frame_id;
  out.width = in.width;
  out.height = in.height;
  out.encoding = in.encoding;
  out.is_bigendian = in.is_bigendian;
  out.step = in.step;
  out.data = in.data;
}

sensor_msgs::msg::Image ToRosMessage(const data::ImageData& in) {
  sensor_msgs::msg::Image out;
  Convert(out, in);
  return out;
}

}  // namespace ausim::converts
