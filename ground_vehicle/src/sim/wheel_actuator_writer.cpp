#include "sim/wheel_actuator_writer.hpp"

#include <stdexcept>
#include <utility>

namespace ground_vehicle {

WheelActuatorWriter::WheelActuatorWriter(WheelActuatorBindingConfig config)
    : config_(std::move(config)),
      actuator_names_(config_.AsArray()) {}

std::string WheelActuatorWriter::ValidateModel(const mjModel* model) const {
  for (const std::string& actuator_name : actuator_names_) {
    if (mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str()) < 0) {
      return "Model is incompatible with the scout controller: missing actuator '" +
             actuator_name + "'";
    }
  }
  return {};
}

void WheelActuatorWriter::Resolve(const mjModel* model) {
  const std::string validation_error = ValidateModel(model);
  if (!validation_error.empty()) {
    throw std::runtime_error(validation_error);
  }

  for (std::size_t i = 0; i < actuator_ids_.size(); ++i) {
    actuator_ids_[i] = mj_name2id(model, mjOBJ_ACTUATOR, actuator_names_[i].c_str());
  }
}

void WheelActuatorWriter::Write(mjData* data, const WheelSpeeds& speeds) const {
  for (std::size_t i = 0; i < actuator_ids_.size(); ++i) {
    data->ctrl[actuator_ids_[i]] = speeds.rad_per_second[i];
  }
}

}  // namespace ground_vehicle
