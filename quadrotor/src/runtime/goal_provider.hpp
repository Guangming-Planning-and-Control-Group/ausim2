#pragma once

#include "config/quadrotor_config.hpp"
#include "runtime/mode_actions_registry.hpp"
#include "runtime/robot_mode_state_machine.hpp"
#include "runtime/quadrotor_runtime_types.hpp"

namespace quadrotor {

struct GoalContext {
  double sim_time = 0.0;
  const State& current_state;
};

using ModeActionsRegistry = ausim::ModeActionsRegistryT<GoalContext>;
using ModeActionCallback = ModeActionsRegistry::Callback;

class GoalProvider {
 public:
  virtual ~GoalProvider() = default;

  virtual GoalReference Evaluate(const GoalContext& context) = 0;
  virtual bool HandleDiscreteCommand(const DiscreteCommand& command, const GoalContext& context) {
    (void)command;
    (void)context;
    return false;
  }
  // Timeout / time-based advancement hook (state-machine Tick, climb ramp, ...).
  virtual void Tick(double dt, const GoalContext& context) {
    (void)dt;
    (void)context;
  }
  virtual RobotModeSnapshot ModeSnapshot() const { return RobotModeSnapshot{}; }
  virtual void Reset() {}
};

class DemoGoalProvider : public GoalProvider {
 public:
  explicit DemoGoalProvider(const QuadrotorConfig& config);

  GoalReference Evaluate(const GoalContext& context) override;
  // Demo provider bypasses the state machine — it always runs an autonomous
  // trajectory. Report AUTO so downstream consumers see a sensible top state.
  RobotModeSnapshot ModeSnapshot() const override;

 private:
  const QuadrotorConfig& config_;
};

class CommandGoalProvider : public GoalProvider {
 public:
  explicit CommandGoalProvider(const QuadrotorConfig& config);

  GoalReference Evaluate(const GoalContext& context) override;
  bool HandleDiscreteCommand(const DiscreteCommand& command, const GoalContext& context) override;
  void Tick(double dt, const GoalContext& context) override;
  RobotModeSnapshot ModeSnapshot() const override;
  void Reset() override;

 private:
  bool HandleTakeoffCommand(const GoalContext& context);
  bool HandleLandCommand(const GoalContext& context);
  bool HandleEmergencyStopCommand(const GoalContext& context);
  static bool MotionCommandActive(const VelocityCommand& command);

  double command_timeout_seconds_ = 0.5;
  SE3Controller::ControlMode control_mode_ = SE3Controller::ControlMode::kVelocity;
  Eigen::Vector3d aircraft_forward_axis_ = Eigen::Vector3d(0.0, 1.0, 0.0);
  ausim::RobotModeActionsConfig mode_actions_;
  ausim::RobotModeStateMachine mode_machine_;
  ModeActionsRegistry actions_;
  bool initialized_ = false;
  bool hold_state_initialized_ = false;
  bool previous_command_valid_ = false;
  Eigen::Vector3d spawn_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero();
  double desired_yaw_ = 0.0;
  double last_sim_time_ = 0.0;
};

}  // namespace quadrotor
