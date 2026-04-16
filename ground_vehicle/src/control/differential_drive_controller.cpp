#include "control/differential_drive_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace ground_vehicle {
namespace {

double ClampSymmetric(double value, double limit) {
  const double positive_limit = std::abs(limit);
  return std::clamp(value, -positive_limit, positive_limit);
}

struct WheelGeometry {
  double longitudinal_offset = 0.0;
  double lateral_offset = 0.0;
  double joint_sign = 1.0;
};

}  // namespace

DifferentialDriveController::DifferentialDriveController(DifferentialDriveConfig config) : config_(config) {
  if (config_.wheel_radius <= 0.0) {
    throw std::runtime_error("ground_vehicle.wheel_radius must be positive.");
  }
  if (config_.track_width <= 0.0) {
    throw std::runtime_error("ground_vehicle.track_width must be positive.");
  }
  if (config_.axle_length <= 0.0) {
    throw std::runtime_error("ground_vehicle.axle_length must be positive.");
  }
  if (config_.max_wheel_speed <= 0.0) {
    throw std::runtime_error("ground_vehicle.max_wheel_speed must be positive.");
  }
}

WheelSpeeds DifferentialDriveController::Compute(double linear_x, double angular_z) const {
  const double v = ClampSymmetric(linear_x, config_.max_linear_speed);
  const double yaw_rate = ClampSymmetric(angular_z, config_.max_angular_speed);
  const double half_axle = config_.axle_length * 0.5;
  const double half_track = config_.track_width * 0.5;
  const auto joint_signs = config_.JointSigns();
  const std::array<WheelGeometry, 4> geometry = {{
      WheelGeometry{half_axle, -half_track, joint_signs[static_cast<int>(WheelIndex::kFrontRight)]},
      WheelGeometry{half_axle, half_track, joint_signs[static_cast<int>(WheelIndex::kFrontLeft)]},
      WheelGeometry{-half_axle, half_track, joint_signs[static_cast<int>(WheelIndex::kRearLeft)]},
      WheelGeometry{-half_axle, -half_track, joint_signs[static_cast<int>(WheelIndex::kRearRight)]},
  }};

  WheelSpeeds speeds;
  for (std::size_t i = 0; i < geometry.size(); ++i) {
    const WheelGeometry& wheel = geometry[i];

    // We intentionally map only body-frame linear.x and angular.z into the
    // wheel rolling direction. The full wheel geometry is kept explicit so the
    // controller can be reused across different wheel tracks, wheelbases and
    // per-wheel transmission sign conventions.
    // ICR 补偿：4 轮 skid-steer 转向时轮子横向滑移会消耗角动量，
    // 需要把用于 yaw 的等效轮速放大 icr_coefficient 倍以抵消滑移损失。
    const double tangential_velocity_x = -yaw_rate * wheel.lateral_offset * config_.icr_coefficient;
    const double tangential_velocity_y = yaw_rate * wheel.longitudinal_offset;
    (void)tangential_velocity_y;

    const double wheel_linear_velocity = v + tangential_velocity_x;
    speeds.rad_per_second[i] = ClampSymmetric(wheel.joint_sign * wheel_linear_velocity / config_.wheel_radius, config_.max_wheel_speed);
  }

  return speeds;
}

}  // namespace ground_vehicle
