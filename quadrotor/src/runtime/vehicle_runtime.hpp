#pragma once

#include <memory>

#include "config/quadrotor_config.hpp"
#include "control/motor_mixer.hpp"
#include "runtime/goal_provider.hpp"

namespace quadrotor {

class VehicleRuntime {
 public:
  explicit VehicleRuntime(const QuadrotorConfig& config);

  RuntimeOutput Step(const RuntimeInput& input, bool recompute_control, double control_dt);
  void Tick(double dt, const RuntimeInput& input);
  bool HandleDiscreteCommand(const DiscreteCommand& command, const RuntimeInput& input);
  RobotModeSnapshot ModeSnapshot() const;
  void Reset();

 private:
  QuadrotorConfig config_;
  SE3Controller controller_;
  MotorMixer mixer_;
  std::unique_ptr<GoalProvider> goal_provider_;
  ControlCommand cached_command_;
  GoalReference cached_goal_;
  Eigen::Vector4d cached_motor_speed_krpm_ = Eigen::Vector4d::Zero();
};

}  // namespace quadrotor
