#include "sim/mujoco_state_reader.hpp"

#include <cmath>

namespace quadrotor {
namespace {

Eigen::Vector3d ReadVector3(const mjData* data, const SensorBinding& binding) {
  if (!binding.resolved || binding.dim < 3) {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(data->sensordata[binding.adr + 0], data->sensordata[binding.adr + 1], data->sensordata[binding.adr + 2]);
}

Eigen::Quaterniond ReadQuaternion(const mjData* data, const SensorBinding& binding) {
  if (!binding.resolved || binding.dim < 4) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(data->sensordata[binding.adr + 0], data->sensordata[binding.adr + 1], data->sensordata[binding.adr + 2],
                            data->sensordata[binding.adr + 3]);
}

double GravityMagnitude(const mjModel* model) {
  return std::sqrt(model->opt.gravity[0] * model->opt.gravity[0] + model->opt.gravity[1] * model->opt.gravity[1] +
                   model->opt.gravity[2] * model->opt.gravity[2]);
}

}  // namespace

RuntimeInput MujocoStateReader::Read(const mjModel* model, const mjData* data, const MujocoBindings& bindings) const {
  RuntimeInput input;
  input.sim_time = data->time;
  input.gravity_magnitude = GravityMagnitude(model);

  if (model->nq >= 3) {
    input.current_state.position = Eigen::Vector3d(data->qpos[0], data->qpos[1], data->qpos[2]);
  }
  if (model->nv >= 3) {
    input.current_state.velocity = Eigen::Vector3d(data->qvel[0], data->qvel[1], data->qvel[2]);
  }

  if (bindings.gyro_sensor().resolved) {
    input.current_state.omega = ReadVector3(data, bindings.gyro_sensor());
    input.imu.angular_velocity = input.current_state.omega;
  } else if (model->nv >= 6) {
    input.current_state.omega = Eigen::Vector3d(data->qvel[3], data->qvel[4], data->qvel[5]);
    input.imu.angular_velocity = input.current_state.omega;
  }

  if (bindings.quaternion_sensor().resolved) {
    input.current_state.quaternion = ReadQuaternion(data, bindings.quaternion_sensor());
  } else if (model->nq >= 7) {
    input.current_state.quaternion = Eigen::Quaterniond(data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]);
  }
  input.current_state.quaternion.normalize();
  input.imu.orientation = input.current_state.quaternion;

  if (bindings.accelerometer_sensor().resolved) {
    input.imu.linear_acceleration = ReadVector3(data, bindings.accelerometer_sensor());
    input.imu.has_linear_acceleration = true;
  }

  return input;
}

}  // namespace quadrotor
