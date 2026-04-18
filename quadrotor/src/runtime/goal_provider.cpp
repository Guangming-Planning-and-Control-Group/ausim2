#include "runtime/goal_provider.hpp"

#include <algorithm>
#include <cmath>

#include "runtime/data_board_interface.hpp"

namespace quadrotor {
namespace {

enum class ExampleMode {
  kSimple = 1,
  kCircular = 2,
};

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

DemoGoalProvider::DemoGoalProvider(const QuadrotorConfig& config) : config_(config) {}

GoalReference DemoGoalProvider::Evaluate(const GoalContext& context) {
  GoalReference goal;
  goal.state.quaternion = Eigen::Quaterniond::Identity();
  goal.forward = config_.hover_goal.heading;
  goal.control_mode = static_cast<SE3Controller::ControlMode>(config_.simulation.control_mode);
  goal.source = "demo";

  const bool velocity_mode = goal.control_mode == SE3Controller::ControlMode::kVelocity;
  const CircleTrajectoryConfig& circle = config_.circle_trajectory;
  if (velocity_mode && context.sim_time < circle.wait_time) {
    goal.state.position = config_.hover_goal.position;
    goal.state.velocity = Eigen::Vector3d::Zero();
    goal.control_mode = SE3Controller::ControlMode::kPosition;
    return goal;
  }

  const auto example_mode = static_cast<ExampleMode>(config_.simulation.example_mode);
  if (example_mode == ExampleMode::kSimple) {
    goal.state.position = config_.hover_goal.position;
    goal.state.velocity = velocity_mode ? config_.hover_goal.velocity : Eigen::Vector3d::Zero();
    if (goal.state.velocity.head<2>().norm() > 1e-6) {
      goal.forward = Eigen::Vector3d(goal.state.velocity.x(), goal.state.velocity.y(), 0.0).normalized();
    }
    return goal;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double phase = 2.0 * kPi * circle.speed_hz * (context.sim_time - circle.wait_time);
  const double cosine = std::cos(phase);
  const double sine = std::sin(phase);

  goal.state.position = Eigen::Vector3d(circle.radius * cosine, circle.radius * sine, circle.height);
  goal.forward = Eigen::Vector3d(-sine, cosine, 0.0);

  if (velocity_mode) {
    const Eigen::Vector2d base_xy(config_.hover_goal.velocity.x(), config_.hover_goal.velocity.y());
    const Eigen::Matrix2d rotation = (Eigen::Matrix2d() << cosine, -sine, sine, cosine).finished();
    const Eigen::Vector2d rotated_xy = rotation * base_xy;
    const double vertical_velocity =
        config_.hover_goal.velocity.z() + circle.height_gain * (config_.hover_goal.position.z() - context.current_state.position.z());

    goal.state.velocity = Eigen::Vector3d(rotated_xy.x(), rotated_xy.y(), vertical_velocity);
    if (rotated_xy.norm() > 1e-6) {
      goal.forward = Eigen::Vector3d(rotated_xy.x(), rotated_xy.y(), 0.0).normalized();
    }
  }

  return goal;
}

RobotModeSnapshot DemoGoalProvider::ModeSnapshot() const {
  // Demo / scripted trajectories run autonomously; expose AUTO so the robot-mode
  // consumers (UI, remote_control) don't misread it as SAFE/on_ground.
  RobotModeSnapshot snapshot;
  snapshot.top_state = ausim::RobotTopLevelState::kAuto;
  snapshot.sub_state = "demo";
  snapshot.accepts_motion = false;
  return snapshot;
}

CommandGoalProvider::CommandGoalProvider(const QuadrotorConfig& config)
    : command_timeout_seconds_(config.ros2.command_timeout),
      control_mode_(static_cast<SE3Controller::ControlMode>(config.simulation.control_mode)),
      aircraft_forward_axis_(config.model.aircraft_forward_axis),
      mode_actions_(config.teleop_mode.actions),
      mode_machine_(config.teleop_mode) {
  actions_.Register("takeoff", [this](const GoalContext& ctx) { return HandleTakeoffCommand(ctx); });
  actions_.Register("land", [this](const GoalContext& ctx) { return HandleLandCommand(ctx); });
  actions_.Register("emergency_stop", [this](const GoalContext& ctx) { return HandleEmergencyStopCommand(ctx); });
}

GoalReference CommandGoalProvider::Evaluate(const GoalContext& context) {
  const std::optional<VelocityCommand> raw_command = ReadFreshVelocityCommand(command_timeout_seconds_);
  if (mode_machine_.enabled()) {
    const bool motion_active = raw_command.has_value() && MotionCommandActive(*raw_command);
    mode_machine_.UpdateConditions({motion_active});
  }
  const std::optional<VelocityCommand> command =
      (!mode_machine_.enabled() || mode_machine_.AcceptsMotion()) ? raw_command : std::optional<VelocityCommand>{};
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
  goal.control_mode = SE3Controller::ControlMode::kPosition;
  goal.state.quaternion = Eigen::Quaterniond::Identity();
  goal.state.omega = Eigen::Vector3d::Zero();
  if (mode_machine_.enabled() && !mode_machine_.Snapshot().sub_state.empty()) {
    goal.source = "teleop:" + mode_machine_.Snapshot().sub_state;
  }

  if (!command.has_value()) {
    // No fresh command:
    // - velocity mode latches the current pose and hovers there
    // - position mode keeps the original spawn-referenced semantics
    if (goal.source.empty()) {
      goal.source = "ros2_hold";
    }
    if (control_mode_ == SE3Controller::ControlMode::kVelocity) {
      if (!hold_state_initialized_ || previous_command_valid_) {
        hold_position_ = context.current_state.position;
        desired_yaw_ = current_yaw;
        hold_state_initialized_ = true;
      }
      goal.state.position = hold_position_;
    } else {
      goal.state.position = spawn_position_;
    }
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

  if (control_mode_ == SE3Controller::ControlMode::kPosition) {
    // cmd_vel.linear  = absolute position offset from spawn (metres)
    // cmd_vel.angular.z = absolute heading (radians, world frame)
    desired_yaw_ = command->angular.z();
    goal.control_mode = SE3Controller::ControlMode::kPosition;
    if (goal.source.empty()) {
      goal.source = "ros2_cmd_vel_pos";
    }
    goal.state.position = spawn_position_ + command->linear;
    goal.state.velocity = Eigen::Vector3d::Zero();
  } else {
    // kVelocity:
    // - cmd_vel.linear is interpreted in the local horizontal odom frame
    //   aligned with the current vehicle heading (x forward, y left, z up)
    // - cmd_vel.angular.z is interpreted as a commanded yaw rate around the
    //   locally held heading. Keeping it at zero should keep the current yaw.
    if (command_reacquired) {
      desired_yaw_ = current_yaw;
    }
    desired_yaw_ += command->angular.z() * dt;
    goal.control_mode = SE3Controller::ControlMode::kVelocity;
    if (goal.source.empty()) {
      goal.source = "ros2_cmd_vel_local";
    }
    goal.state.position = context.current_state.position;
    goal.state.velocity = RotateLocalVelocityToWorld(command->linear, current_yaw);
    goal.state.omega = Eigen::Vector3d(0.0, 0.0, command->angular.z());
  }

  goal.forward = ForwardFromYaw(desired_yaw_);

  return goal;
}

bool CommandGoalProvider::HandleDiscreteCommand(const DiscreteCommand& command, const GoalContext& context) {
  if (command.event_name.empty()) {
    return false;
  }
  if (mode_machine_.enabled()) {
    return mode_machine_.HandleEvent(
        command.event_name,
        {.execute_action = [this, &context](std::string_view action_name) { return actions_.Invoke(action_name, context); }});
  }
  // No state machine configured — run the action directly (legacy path).
  return actions_.Invoke(command.event_name, context);
}

void CommandGoalProvider::Tick(double dt, const GoalContext& context) {
  if (!mode_machine_.enabled()) {
    return;
  }
  mode_machine_.Tick(
      dt, {.execute_action = [this, &context](std::string_view action_name) { return actions_.Invoke(action_name, context); }});
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
  return true;
}

void CommandGoalProvider::Reset() {
  mode_machine_.Reset();
  initialized_ = false;
  hold_state_initialized_ = false;
  previous_command_valid_ = false;
  desired_yaw_ = 0.0;
  last_sim_time_ = 0.0;
  spawn_position_ = Eigen::Vector3d::Zero();
  hold_position_ = Eigen::Vector3d::Zero();
}

bool CommandGoalProvider::MotionCommandActive(const VelocityCommand& command) {
  return command.linear.norm() > 1e-3 || command.angular.norm() > 1e-3;
}

}  // namespace quadrotor
