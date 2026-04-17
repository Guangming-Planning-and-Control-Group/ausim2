#include "runtime/vehicle_runtime.hpp"

namespace quadrotor {

VehicleRuntime::VehicleRuntime(const QuadrotorConfig& config)
    : config_(config),
      mixer_(MixerParams{config_.vehicle.Ct, config_.vehicle.Cd, config_.vehicle.arm_length, config_.vehicle.max_thrust, config_.vehicle.max_torque,
                         config_.vehicle.max_speed_krpm}) {
  controller_.kx = config_.controller.kx;
  controller_.kv = config_.controller.kv;
  controller_.kR = config_.controller.kR;
  controller_.kw = config_.controller.kw;
  controller_.control_mode = static_cast<SE3Controller::ControlMode>(config_.simulation.control_mode);
  controller_.setAircraftForwardAxis(config_.model.aircraft_forward_axis);

  if (config_.simulation.example_mode == 0) {
    goal_provider_ = std::make_unique<CommandGoalProvider>(config_);
  } else {
    goal_provider_ = std::make_unique<DemoGoalProvider>(config_);
  }
}

RuntimeOutput VehicleRuntime::Step(const RuntimeInput& input, bool recompute_control, double control_dt) {
  if (recompute_control) {
    cached_goal_ = goal_provider_->Evaluate(GoalContext{input.sim_time, input.current_state});
    controller_.control_mode = cached_goal_.control_mode;
    cached_command_ = controller_.controlUpdate(input.current_state, cached_goal_.state, control_dt, cached_goal_.forward);
  }

  const double mixer_thrust = cached_command_.thrust * input.gravity_magnitude * config_.vehicle.mass;
  const Eigen::Vector3d mixer_torque = cached_command_.angular * config_.torque_scale;
  cached_motor_speed_krpm_ = mixer_.calculate(mixer_thrust, mixer_torque.x(), mixer_torque.y(), mixer_torque.z());

  RuntimeOutput output;
  output.goal = cached_goal_;
  output.command = cached_command_;
  output.motor_speed_krpm = cached_motor_speed_krpm_;
  return output;
}

bool VehicleRuntime::HandleDiscreteCommand(const DiscreteCommand& command, const RuntimeInput& input) {
  if (!goal_provider_) {
    return false;
  }
  return goal_provider_->HandleDiscreteCommand(DiscreteCommand{command}, GoalContext{input.sim_time, input.current_state});
}

RobotModeSnapshot VehicleRuntime::ModeSnapshot() const {
  if (!goal_provider_) {
    return RobotModeSnapshot{};
  }
  return goal_provider_->ModeSnapshot();
}

void VehicleRuntime::Reset() {
  cached_command_ = ControlCommand{};
  cached_goal_ = GoalReference{};
  cached_motor_speed_krpm_.setZero();
  if (goal_provider_) {
    goal_provider_->Reset();
  }
}

}  // namespace quadrotor
