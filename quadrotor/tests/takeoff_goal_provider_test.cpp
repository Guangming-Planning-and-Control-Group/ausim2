#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "config/quadrotor_config.hpp"
#include "runtime/data_board_interface.hpp"
#include "runtime/goal_provider.hpp"
#include "runtime/runtime_types.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

quadrotor::CommandGoalProvider BuildProvider() {
  ausim::QuadrotorConfig config;
  config.ros2.command_timeout = 0.5;
  config.model.aircraft_forward_axis = Eigen::Vector3d::UnitX();
  config.teleop_mode.initial_state = "on_ground";
  config.teleop_mode.actions.takeoff.height = 1.0;
  config.teleop_mode.states = {
      {"on_ground", "SAFE", false},
      {"hover", "MANUAL_READY", true},
      {"velocity_control", "MANUAL_ACTIVE", true},
  };
  config.teleop_mode.transitions = {
      {"on_ground", "hover", "takeoff", "", "", "takeoff", 0.0},
      {"hover", "velocity_control", "", "motion_active", "", "", 0.0},
      {"velocity_control", "hover", "", "motion_inactive", "", "", 0.0},
  };
  return quadrotor::CommandGoalProvider(config);
}

quadrotor::State BuildState() {
  quadrotor::State state;
  state.position = Eigen::Vector3d(0.0, 0.0, 0.01);
  state.quaternion = Eigen::Quaterniond::Identity();
  return state;
}

quadrotor::DiscreteCommand TakeoffCommand() {
  quadrotor::DiscreteCommand takeoff;
  takeoff.event_name = "takeoff";
  takeoff.kind = quadrotor::DiscreteCommandKind::kGenericEvent;
  takeoff.sequence = 1;
  return takeoff;
}

void WriteVelocityCommand(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular = Eigen::Vector3d::Zero()) {
  quadrotor::VelocityCommand command;
  command.linear = linear;
  command.angular = angular;
  command.received_time = std::chrono::steady_clock::now();
  quadrotor::WriteVelocityCommand(command);
}

}  // namespace

int main() {
  quadrotor::ClearVelocityCommand();

  quadrotor::CommandGoalProvider provider = BuildProvider();
  const quadrotor::State current_state = BuildState();
  const quadrotor::GoalContext context{0.0, current_state};

  const bool handled = provider.HandleDiscreteCommand(TakeoffCommand(), context);
  Expect(handled, "takeoff command should be handled");

  WriteVelocityCommand(Eigen::Vector3d::Zero());

  const quadrotor::GoalReference goal = provider.Evaluate(context);
  Expect(std::abs(goal.state.position.z() - 1.0) < 1e-9, "takeoff target height should remain 1.0 under fresh zero cmd_vel");

  quadrotor::ClearVelocityCommand();

  quadrotor::CommandGoalProvider integrator_provider = BuildProvider();
  const bool integrator_takeoff_handled = integrator_provider.HandleDiscreteCommand(TakeoffCommand(), context);
  Expect(integrator_takeoff_handled, "takeoff command should be handled before integrating cmd_vel");

  WriteVelocityCommand(Eigen::Vector3d(0.5, 0.0, 0.0));
  (void)integrator_provider.Evaluate(context);
  const quadrotor::GoalReference integrated_goal = integrator_provider.Evaluate(quadrotor::GoalContext{1.0, current_state});

  Expect(std::abs(integrated_goal.state.position.x() - 0.5) < 1e-9, "cmd_vel linear.x should integrate into target x position");
  Expect(std::abs(integrated_goal.state.position.y()) < 1e-9, "cmd_vel integration should not drift target y position");
  Expect(std::abs(integrated_goal.state.position.z() - 1.0) < 1e-9, "cmd_vel integration should preserve takeoff target height");
  Expect(integrated_goal.state.velocity.norm() < 1e-9, "integrated cmd_vel should feed zero velocity to the position controller");

  quadrotor::ClearVelocityCommand();
  return 0;
}
