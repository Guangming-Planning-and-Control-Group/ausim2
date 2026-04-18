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
      ausim::RobotModeTransitionConfig{"on_ground", "hover", "takeoff", "", "", "takeoff", 0.0},
      ausim::RobotModeTransitionConfig{"hover", "velocity_control", "", "motion_active", "", "", 0.0},
      ausim::RobotModeTransitionConfig{"velocity_control", "hover", "", "motion_inactive", "", "", 0.0},
      // 1.5s timeout in hover returns to on_ground via the `land` action.
      ausim::RobotModeTransitionConfig{"hover", "on_ground", "", "", "", "land", 1.5},
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

  const auto record_action = [&executed_actions](std::string_view action_name) {
    executed_actions.emplace_back(action_name);
    return true;
  };

  const bool takeoff_transitioned = machine.HandleEvent("takeoff", {.execute_action = record_action});
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

  // Reset event name is not registered on any transition -> no-op.
  const bool unexpected_event = machine.HandleEvent("reset");
  Expect(!unexpected_event, "unexpected reset event should not transition");

  // Unknown events are harmlessly ignored (no throw, no transition).
  const bool ignored = machine.HandleEvent("not_a_registered_event");
  Expect(!ignored, "unregistered event should be ignored");

  // Timeout transition: advancing time past the 1.5s threshold should fire the
  // hover -> on_ground transition with the `land` action.
  executed_actions.clear();
  Expect(!machine.Tick(0.5, {.execute_action = record_action}), "0.5s is below the 1.5s timeout");
  Expect(machine.Snapshot().sub_state == "hover", "still in hover after partial tick");
  const bool timeout_fired = machine.Tick(1.1, {.execute_action = record_action});
  Expect(timeout_fired, "total 1.6s elapsed should fire hover timeout");
  Expect(machine.Snapshot().sub_state == "on_ground", "timeout should take state back to on_ground");
  Expect(executed_actions.size() == 1 && executed_actions.front() == "land", "timeout action should run `land`");

  return 0;
}
