#include "runtime/goal_provider.hpp"

#include <algorithm>
#include <cmath>

#include "runtime/data_board_interface.hpp"

namespace quadrotor {
namespace {

double AircraftHeadingFromQuaternion(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& aircraft_forward_axis) {
  const Eigen::Vector3d forward_world = quaternion.normalized() * aircraft_forward_axis;
  return std::atan2(forward_world.y(), forward_world.x());
}

Eigen::Vector3d ForwardFromYaw(double yaw) { return Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0); }

Eigen::Vector3d RotateLocalVelocityToWorld(const Eigen::Vector3d& local_velocity, double yaw) {
  const Eigen::Matrix2d yaw_rotation = (Eigen::Matrix2d() << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw)).finished();
  const Eigen::Vector2d world_xy = yaw_rotation * local_velocity.head<2>();
  return Eigen::Vector3d(world_xy.x(), world_xy.y(), local_velocity.z());
}

}  // namespace

CommandGoalProvider::CommandGoalProvider(const QuadrotorConfig& config)
    : command_timeout_seconds_(config.ros2.command_timeout),
      aircraft_forward_axis_(config.model.aircraft_forward_axis),
      mode_actions_(config.teleop_mode.actions),
      mode_machine_(config.teleop_mode) {
  actions_.Register("takeoff", [this](const GoalContext& ctx) { return HandleTakeoffCommand(ctx); });
  actions_.Register("land", [this](const GoalContext& ctx) { return HandleLandCommand(ctx); });
  actions_.Register("emergency_stop", [this](const GoalContext& ctx) { return HandleEmergencyStopCommand(ctx); });
}

GoalReference CommandGoalProvider::Evaluate(const GoalContext& context) {
  const std::optional<VelocityCommand> raw_command = ReadFreshVelocityCommand(command_timeout_seconds_);
  const bool motion_active = raw_command.has_value() && MotionCommandActive(*raw_command);
  if (mode_machine_.enabled()) {
    mode_machine_.UpdateConditions({motion_active});
  }
  const std::optional<VelocityCommand> command =
      ((!mode_machine_.enabled() || mode_machine_.AcceptsMotion()) && motion_active) ? raw_command : std::optional<VelocityCommand>{};
  const double current_yaw = AircraftHeadingFromQuaternion(context.current_state.quaternion, aircraft_forward_axis_);

  // Capture spawn state on first call.
  if (!initialized_) {
    spawn_position_ = context.current_state.position;
    hold_position_ = context.current_state.position;
    desired_yaw_ = current_yaw;
    last_sim_time_ = context.sim_time;
    initialized_ = true;
    hold_state_initialized_ = true;
  }

  GoalReference goal;
  goal.state.quaternion = Eigen::Quaterniond::Identity();
  goal.state.omega = Eigen::Vector3d::Zero();
  if (mode_machine_.enabled() && !mode_machine_.Snapshot().sub_state.empty()) {
    goal.source = "teleop:" + mode_machine_.Snapshot().sub_state;
  }

  if (!command.has_value()) {
    if (goal.source.empty()) {
      goal.source = "ros2_hold";
    }
    if (!action_hold_active_ && (!hold_state_initialized_ || previous_command_valid_)) {
      hold_position_ = context.current_state.position;
      desired_yaw_ = current_yaw;
      hold_state_initialized_ = true;
    }
    goal.state.position = hold_position_;
    goal.state.velocity = Eigen::Vector3d::Zero();
    goal.forward = ForwardFromYaw(desired_yaw_);
    last_sim_time_ = context.sim_time;
    previous_command_valid_ = false;
    return goal;
  }

  const double dt = std::max(0.0, context.sim_time - last_sim_time_);
  const bool command_reacquired = !previous_command_valid_;
  last_sim_time_ = context.sim_time;
  previous_command_valid_ = true;
  action_hold_active_ = false;

  if (command_reacquired) {
    desired_yaw_ = current_yaw;
  }
  hold_position_ += RotateLocalVelocityToWorld(command->linear, current_yaw) * dt;
  desired_yaw_ += command->angular.z() * dt;
  if (goal.source.empty()) {
    goal.source = "ros2_cmd_vel_integrated";
  }
  goal.state.position = hold_position_;
  goal.state.velocity = Eigen::Vector3d::Zero();

  goal.forward = ForwardFromYaw(desired_yaw_);

  return goal;
}

bool CommandGoalProvider::HandleDiscreteCommand(const DiscreteCommand& command, const GoalContext& context) {
  if (command.event_name.empty()) {
    return false;
  }
  if (mode_machine_.enabled()) {
    return mode_machine_.HandleEvent(
        command.event_name, {.execute_action = [this, &context](std::string_view action_name) { return actions_.Invoke(action_name, context); }});
  }
  // No state machine configured — run the action directly (legacy path).
  return actions_.Invoke(command.event_name, context);
}

void CommandGoalProvider::Tick(double dt, const GoalContext& context) {
  if (!mode_machine_.enabled()) {
    return;
  }
  mode_machine_.Tick(dt, {.execute_action = [this, &context](std::string_view action_name) { return actions_.Invoke(action_name, context); }});
}

RobotModeSnapshot CommandGoalProvider::ModeSnapshot() const { return mode_machine_.Snapshot(); }

bool CommandGoalProvider::HandleTakeoffCommand(const GoalContext& context) {
  const double current_yaw = AircraftHeadingFromQuaternion(context.current_state.quaternion, aircraft_forward_axis_);
  if (!initialized_) {
    spawn_position_ = context.current_state.position;
    initialized_ = true;
  }

  hold_position_ = context.current_state.position;
  // climb_rate > 0 is where a ramped takeoff would live; currently snap to height.
  // TODO(mode_actions): interpolate hold_position_.z() toward takeoff.height at climb_rate.
  hold_position_.z() = mode_actions_.takeoff.height;
  desired_yaw_ = current_yaw;
  last_sim_time_ = context.sim_time;
  hold_state_initialized_ = true;
  previous_command_valid_ = false;
  action_hold_active_ = true;
  return true;
}

bool CommandGoalProvider::HandleLandCommand(const GoalContext& context) {
  const double current_yaw = AircraftHeadingFromQuaternion(context.current_state.quaternion, aircraft_forward_axis_);
  if (!initialized_) {
    spawn_position_ = context.current_state.position;
    initialized_ = true;
  }
  hold_position_ = context.current_state.position;
  // descent_rate > 0 is where a ramped land would live; currently snap to spawn z.
  // TODO(mode_actions): interpolate hold_position_.z() toward spawn_position_.z() at descent_rate.
  hold_position_.z() = spawn_position_.z();
  desired_yaw_ = current_yaw;
  last_sim_time_ = context.sim_time;
  hold_state_initialized_ = true;
  previous_command_valid_ = false;
  action_hold_active_ = true;
  return true;
}

bool CommandGoalProvider::HandleEmergencyStopCommand(const GoalContext& context) {
  // Skeleton: clamp goal to current position/zero velocity. Actual motor-cut
  // behaviour is delegated to the controller / sim layer in a later iteration.
  const double current_yaw = AircraftHeadingFromQuaternion(context.current_state.quaternion, aircraft_forward_axis_);
  hold_position_ = context.current_state.position;
  desired_yaw_ = current_yaw;
  last_sim_time_ = context.sim_time;
  hold_state_initialized_ = true;
  previous_command_valid_ = false;
  action_hold_active_ = true;
  return true;
}

void CommandGoalProvider::Reset() {
  mode_machine_.Reset();
  initialized_ = false;
  hold_state_initialized_ = false;
  previous_command_valid_ = false;
  action_hold_active_ = false;
  desired_yaw_ = 0.0;
  last_sim_time_ = 0.0;
  spawn_position_ = Eigen::Vector3d::Zero();
  hold_position_ = Eigen::Vector3d::Zero();
}

bool CommandGoalProvider::MotionCommandActive(const VelocityCommand& command) {
  return command.linear.norm() > 1e-3 || command.angular.norm() > 1e-3;
}

}  // namespace quadrotor
