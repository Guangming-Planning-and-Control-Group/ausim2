#pragma once

#include "config/quadrotor_config.hpp"
#include "runtime/robot_mode_state_machine.hpp"
#include "runtime/quadrotor_runtime_types.hpp"

namespace quadrotor {

struct GoalContext {
  double sim_time = 0.0;
  const State& current_state;
};

class GoalProvider {
 public:
  virtual ~GoalProvider() = default;

  virtual GoalReference Evaluate(const GoalContext& context) = 0;
  virtual bool HandleDiscreteCommand(const DiscreteCommand& command, const GoalContext& context) {
    (void)command;
    (void)context;
    return false;
  }
  virtual RobotModeSnapshot ModeSnapshot() const { return RobotModeSnapshot{}; }
  virtual void Reset() {}
};

class DemoGoalProvider : public GoalProvider {
 public:
  explicit DemoGoalProvider(const QuadrotorConfig& config);

  GoalReference Evaluate(const GoalContext& context) override;

 private:
  const QuadrotorConfig& config_;
};

class CommandGoalProvider : public GoalProvider {
 public:
  explicit CommandGoalProvider(const QuadrotorConfig& config);

  GoalReference Evaluate(const GoalContext& context) override;
  bool HandleDiscreteCommand(const DiscreteCommand& command, const GoalContext& context) override;
  RobotModeSnapshot ModeSnapshot() const override;
  void Reset() override;

 private:
  bool HandleTakeoffCommand(const GoalContext& context);
  static bool MotionCommandActive(const VelocityCommand& command);

  double command_timeout_seconds_ = 0.5;
  SE3Controller::ControlMode control_mode_ = SE3Controller::ControlMode::kVelocity;
  Eigen::Vector3d aircraft_forward_axis_ = Eigen::Vector3d(0.0, 1.0, 0.0);
  double takeoff_height_ = 0.3;
  RobotModeStateMachine mode_machine_;
  bool initialized_ = false;
  bool hold_state_initialized_ = false;
  bool previous_command_valid_ = false;
  Eigen::Vector3d spawn_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero();
  double desired_yaw_ = 0.0;
  double last_sim_time_ = 0.0;
};

}  // namespace quadrotor
