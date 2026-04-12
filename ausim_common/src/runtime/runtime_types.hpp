#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller/state.hpp"

namespace ausim {

struct ImuData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  bool has_linear_acceleration = false;
};

struct RuntimeInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double sim_time = 0.0;
  double gravity_magnitude = 9.81;
  State current_state;
  ImuData imu;
};

struct VelocityCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  std::chrono::steady_clock::time_point received_time = std::chrono::steady_clock::now();
};

struct TelemetrySnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double sim_time = 0.0;
  State state;
  ImuData imu;
  State goal_state;
  Eigen::Vector3d forward = Eigen::Vector3d(1.0, 0.0, 0.0);
  Eigen::Vector4d motor_speed_krpm = Eigen::Vector4d::Zero();
  std::string goal_source;
  bool has_goal = false;
};

enum class CameraFrameFormat : std::uint32_t {
  kRgb8 = 0,
  kDepth32F = 1,
};

constexpr std::uint32_t CameraFrameBytesPerPixel(CameraFrameFormat format) {
  switch (format) {
    case CameraFrameFormat::kRgb8:
      return 3;
    case CameraFrameFormat::kDepth32F:
      return 4;
  }
  return 0;
}

struct CameraFrame {
  double sim_time = 0.0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  std::uint32_t sequence = 0;
  CameraFrameFormat format = CameraFrameFormat::kRgb8;
  std::vector<std::uint8_t> data;
};

}  // namespace ausim

namespace quadrotor {

using ::ausim::CameraFrame;
using ::ausim::CameraFrameBytesPerPixel;
using ::ausim::CameraFrameFormat;
using ::ausim::ImuData;
using ::ausim::RuntimeInput;
using ::ausim::TelemetrySnapshot;
using ::ausim::VelocityCommand;

}  // namespace quadrotor
