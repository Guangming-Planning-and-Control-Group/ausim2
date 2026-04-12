#pragma once

#include <mujoco/mujoco.h>

#include "config/quadrotor_config.hpp"
#include "sim/mujoco_bindings.hpp"

namespace quadrotor {

class MujocoActuatorWriter {
 public:
  explicit MujocoActuatorWriter(const VehicleParams& params);

  void WriteMotorInputs(const MujocoBindings& bindings, mjData* data, const Eigen::Vector4d& motor_speed_krpm) const;

 private:
  double CalcMotorForce(double krpm) const;
  double CalcMotorInput(double krpm) const;

  VehicleParams params_;
};

}  // namespace quadrotor
