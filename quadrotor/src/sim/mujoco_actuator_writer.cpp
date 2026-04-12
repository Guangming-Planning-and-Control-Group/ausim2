#include "sim/mujoco_actuator_writer.hpp"

#include <algorithm>

namespace quadrotor {

MujocoActuatorWriter::MujocoActuatorWriter(const VehicleParams& params) : params_(params) {}

void MujocoActuatorWriter::WriteMotorInputs(const MujocoBindings& bindings, mjData* data, const Eigen::Vector4d& motor_speed_krpm) const {
  const std::array<int, 4>& actuator_ids = bindings.motor_actuator_ids();
  for (std::size_t i = 0; i < actuator_ids.size(); ++i) {
    data->ctrl[actuator_ids[i]] = CalcMotorInput(motor_speed_krpm[static_cast<Eigen::Index>(i)]);
  }
}

double MujocoActuatorWriter::CalcMotorForce(double krpm) const { return params_.Ct * krpm * krpm; }

double MujocoActuatorWriter::CalcMotorInput(double krpm) const {
  const double clamped_speed = std::clamp(krpm, 0.0, params_.max_speed_krpm);
  const double force = CalcMotorForce(clamped_speed);
  const double input = force / params_.max_thrust;
  return std::clamp(input, 0.0, 1.0);
}

}  // namespace quadrotor
