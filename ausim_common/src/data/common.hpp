#pragma once

#include <string>

namespace ausim::data {

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Header {
  double stamp_seconds = 0.0;
  std::string frame_id;
};

struct Pose {
  Vector3 position;
  Quaternion orientation;
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

}  // namespace ausim::data
