#pragma once

#include <string>

#include <Eigen/Core>

#include "runtime/runtime_types.hpp"

namespace quadrotor {

struct GoalReference {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  State state;
  Eigen::Vector3d forward = Eigen::Vector3d(1.0, 0.0, 0.0);
  std::string source;
};

struct RuntimeOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GoalReference goal;
  ControlCommand command;
  Eigen::Vector4d motor_speed_krpm = Eigen::Vector4d::Zero();
};

}  // namespace quadrotor
