#pragma once

#include <array>
#include <string>

#include <mujoco/mujoco.h>

#include "config/scout_config.hpp"
#include "control/differential_drive_controller.hpp"

namespace ground_vehicle {

class WheelActuatorWriter {
 public:
  explicit WheelActuatorWriter(WheelActuatorBindingConfig config);

  std::string ValidateModel(const mjModel* model) const;
  void Resolve(const mjModel* model);
  void Write(mjData* data, const WheelSpeeds& speeds) const;

 private:
  WheelActuatorBindingConfig config_;
  std::array<std::string, 4> actuator_names_;
  std::array<int, 4> actuator_ids_ = {-1, -1, -1, -1};
};

}  // namespace ground_vehicle
