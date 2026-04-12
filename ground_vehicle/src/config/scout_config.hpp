#pragma once

#include <array>
#include <filesystem>
#include <string>

#include "config/quadrotor_config.hpp"

namespace ground_vehicle {

enum class WheelIndex {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

struct WheelActuatorBindingConfig {
  std::string front_right = "velocity_front_right_wheel";
  std::string front_left = "velocity_front_left_wheel";
  std::string rear_left = "velocity_rear_left_wheel";
  std::string rear_right = "velocity_rear_right_wheel";

  std::array<std::string, 4> AsArray() const {
    return {front_right, front_left, rear_left, rear_right};
  }
};

struct ScoutBindingConfig {
  std::string freejoint_name = "floating_base_joint";
  WheelActuatorBindingConfig wheel_actuators;
};

struct DifferentialDriveConfig {
  double wheel_radius = 0.165;
  double track_width = 0.58306;
  double axle_length = 0.498;
  double max_linear_speed = 1.5;
  double max_angular_speed = 2.0;
  double max_wheel_speed = 20.0;
  double front_right_joint_sign = 1.0;
  double front_left_joint_sign = -1.0;
  double rear_left_joint_sign = -1.0;
  double rear_right_joint_sign = 1.0;

  std::array<double, 4> JointSigns() const {
    return {
        front_right_joint_sign,
        front_left_joint_sign,
        rear_left_joint_sign,
        rear_right_joint_sign,
    };
  }
};

struct ScoutConfig {
  ausim::SimConfig common;
  ScoutBindingConfig bindings;
  DifferentialDriveConfig drive;
};

ScoutConfig LoadScoutConfigFromYaml(const std::string& path);
ScoutConfig LoadScoutConfigFromYaml(
    const std::string& sim_config_path,
    const std::string& robot_config_path);

}  // namespace ground_vehicle
