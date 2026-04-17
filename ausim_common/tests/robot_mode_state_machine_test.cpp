#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "config/quadrotor_config.hpp"
#include "runtime/robot_mode_state_machine.hpp"

namespace {

ausim::RobotModeConfig BuildConfig() {
  ausim::RobotModeConfig config;
  config.initial_state = "on_ground";
  config.states = {
      ausim::RobotModeStateConfig{"on_ground", "SAFE", false},
      ausim::RobotModeStateConfig{"hover", "MANUAL_READY", true},
      ausim::RobotModeStateConfig{"velocity_control", "MANUAL_ACTIVE", true},
  };
  config.transitions = {
      ausim::RobotModeTransitionConfig{"on_ground", "hover", "takeoff", "", "", "takeoff"},
      ausim::RobotModeTransitionConfig{"hover", "velocity_control", "", "motion_active", "", ""},
      ausim::RobotModeTransitionConfig{"velocity_control", "hover", "", "motion_inactive", "", ""},
  };
  return config;
}

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  std::vector<std::string> executed_actions;

  ausim::RobotModeStateMachine machine(BuildConfig());
  Expect(machine.Snapshot().sub_state == "on_ground", "expected initial state to be on_ground");
  Expect(!machine.AcceptsMotion(), "on_ground should not accept motion");

  const bool takeoff_transitioned =
      machine.HandleEvent(ausim::DiscreteCommandKind::kTakeoff,
                          {.execute_action = [&executed_actions](std::string_view action_name) {
                             executed_actions.emplace_back(action_name);
                             return action_name == "takeoff";
                           }});
  Expect(takeoff_transitioned, "takeoff event should trigger a transition");
  Expect(machine.Snapshot().sub_state == "hover", "takeoff should move state machine to hover");
  Expect(machine.Snapshot().top_state == ausim::RobotTopLevelState::kManualReady, "hover should map to MANUAL_READY");
  Expect(machine.AcceptsMotion(), "hover should accept motion");
  Expect(executed_actions.size() == 1 && executed_actions.front() == "takeoff", "takeoff action should execute exactly once");

  const bool motion_started = machine.UpdateConditions({.motion_active = true});
  Expect(motion_started, "motion_active should transition into velocity_control");
  Expect(machine.Snapshot().sub_state == "velocity_control", "motion_active should move to velocity_control");
  Expect(machine.Snapshot().top_state == ausim::RobotTopLevelState::kManualActive, "velocity_control should map to MANUAL_ACTIVE");

  const bool motion_stopped = machine.UpdateConditions({.motion_active = false});
  Expect(motion_stopped, "motion_inactive should transition back to hover");
  Expect(machine.Snapshot().sub_state == "hover", "motion_inactive should move back to hover");

  const bool unexpected_event = machine.HandleEvent(ausim::DiscreteCommandKind::kResetSimulation, {});
  Expect(!unexpected_event, "unexpected reset event should not transition");

  return 0;
}
