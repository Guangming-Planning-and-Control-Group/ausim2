#pragma once

#include <array>

#include "config/scout_config.hpp"

namespace ground_vehicle {

struct WheelSpeeds {
  std::array<double, 4> rad_per_second = {0.0, 0.0, 0.0, 0.0};
};

class DifferentialDriveController {
 public:
  explicit DifferentialDriveController(DifferentialDriveConfig config);

  WheelSpeeds Compute(double linear_x, double angular_z) const;

 private:
  DifferentialDriveConfig config_;
};

}  // namespace ground_vehicle
