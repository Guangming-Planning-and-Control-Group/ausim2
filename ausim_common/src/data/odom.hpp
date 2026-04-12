#pragma once

#include <string>

#include "data/common.hpp"

namespace ausim::data {

struct OdomData {
  Header header;
  std::string child_frame_id;
  Pose pose;
  Twist twist;
};

}  // namespace ausim::data
