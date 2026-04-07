#pragma once

#include <optional>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller/state.hpp"

namespace quadrotor {

class SE3Controller {
 public:
  enum class ControlMode {
    kDirect = 0,
    kVelocity = 1,
    kPosition = 2,
  };

  double kx = 0.0;
  double kv = 0.0;
  double kR = 0.0;
  double kw = 0.0;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -1.0);
  ControlMode control_mode = ControlMode::kPosition;

  void setCurrentState(const State& state);
  void setGoalState(const State& state);

  ControlCommand controlUpdate(
      const State& current_state,
      const State& goal_state,
      double dt,
      const Eigen::Vector3d& forward);

 private:
  struct AngularError {
    Eigen::Vector3d attitude = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
    double thrust = 0.0;
  };

  std::pair<Eigen::Vector3d, Eigen::Vector3d> updateLinearError() const;
  AngularError updateAngularError(
      const Eigen::Vector3d& trans_control,
      const Eigen::Vector3d& forward) const;

  std::optional<State> goal_state_;
  std::optional<State> current_state_;
};

}  // namespace quadrotor
