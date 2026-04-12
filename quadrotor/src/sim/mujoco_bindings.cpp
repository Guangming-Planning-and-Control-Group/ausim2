#include "sim/mujoco_bindings.hpp"

#include <stdexcept>
#include <utility>

namespace quadrotor {

MujocoBindings::MujocoBindings(const QuadrotorConfig& config)
    : motor_names_(config.actuators.motor_names),
      vehicle_body_name_(config.model.vehicle_body_name),
      track_camera_name_(config.simulation.track_camera_name),
      gyro_sensor_{config.state.gyro_sensor_name},
      accelerometer_sensor_{config.state.accelerometer_sensor_name},
      quaternion_sensor_{config.state.quaternion_sensor_name} {}

std::string MujocoBindings::ValidateModel(const mjModel* model) const {
  for (const std::string& motor_name : motor_names_) {
    if (mj_name2id(model, mjOBJ_ACTUATOR, motor_name.c_str()) < 0) {
      return "Model is incompatible with the quadrotor controller: missing actuator '" + motor_name + "'";
    }
  }
  return {};
}

void MujocoBindings::Resolve(const mjModel* model) {
  const std::string validation_error = ValidateModel(model);
  if (!validation_error.empty()) {
    throw std::runtime_error(validation_error);
  }

  for (std::size_t i = 0; i < motor_actuator_ids_.size(); ++i) {
    motor_actuator_ids_[i] = mj_name2id(model, mjOBJ_ACTUATOR, motor_names_[i].c_str());
  }

  vehicle_body_id_ = mj_name2id(model, mjOBJ_BODY, vehicle_body_name_.c_str());
  if (vehicle_body_id_ < 0 && model->nbody > 1) {
    vehicle_body_id_ = 1;
  }

  track_camera_id_ = track_camera_name_.empty() ? -1 : mj_name2id(model, mjOBJ_CAMERA, track_camera_name_.c_str());

  ResolveSensor(model, &gyro_sensor_);
  ResolveSensor(model, &accelerometer_sensor_);
  ResolveSensor(model, &quaternion_sensor_);
}

void MujocoBindings::ResolveSensor(const mjModel* model, SensorBinding* binding) {
  binding->id = -1;
  binding->adr = -1;
  binding->dim = 0;
  binding->resolved = false;

  if (binding->name.empty()) {
    return;
  }

  const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, binding->name.c_str());
  if (sensor_id < 0) {
    return;
  }

  binding->id = sensor_id;
  binding->adr = model->sensor_adr[sensor_id];
  binding->dim = model->sensor_dim[sensor_id];
  binding->resolved = true;
}

}  // namespace quadrotor
