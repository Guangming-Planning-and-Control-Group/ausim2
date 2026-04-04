#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace quadrotor {

struct State {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();
};

struct ControlCommand {
  double thrust = 0.0;
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

}  // namespace quadrotor
