#include "control/differential_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ground_vehicle {
namespace {

double ClampSymmetric(double value, double limit) {
  const double positive_limit = std::abs(limit);
  return std::clamp(value, -positive_limit, positive_limit);
}

}  // namespace

DifferentialDriveController::DifferentialDriveController(DifferentialDriveConfig config)
    : config_(config) {
  if (config_.wheel_radius <= 0.0) {
    throw std::runtime_error("ground_vehicle.wheel_radius must be positive.");
  }
  if (config_.track_width <= 0.0) {
    throw std::runtime_error("ground_vehicle.track_width must be positive.");
  }
  if (config_.max_wheel_speed <= 0.0) {
    throw std::runtime_error("ground_vehicle.max_wheel_speed must be positive.");
  }
}

WheelSpeeds DifferentialDriveController::Compute(double linear_x, double angular_z) const {
  const double v = ClampSymmetric(linear_x, config_.max_linear_speed);
  const double yaw_rate = ClampSymmetric(angular_z, config_.max_angular_speed);

  const double left_linear = v - yaw_rate * config_.track_width * 0.5;
  const double right_linear = v + yaw_rate * config_.track_width * 0.5;

  const double left_wheel =
      ClampSymmetric(config_.left_joint_sign * left_linear / config_.wheel_radius,
                     config_.max_wheel_speed);
  const double right_wheel =
      ClampSymmetric(config_.right_joint_sign * right_linear / config_.wheel_radius,
                     config_.max_wheel_speed);

  WheelSpeeds speeds;
  speeds.rad_per_second[static_cast<int>(WheelIndex::kFrontRight)] = right_wheel;
  speeds.rad_per_second[static_cast<int>(WheelIndex::kFrontLeft)] = left_wheel;
  speeds.rad_per_second[static_cast<int>(WheelIndex::kRearLeft)] = left_wheel;
  speeds.rad_per_second[static_cast<int>(WheelIndex::kRearRight)] = right_wheel;
  return speeds;
}

}  // namespace ground_vehicle
