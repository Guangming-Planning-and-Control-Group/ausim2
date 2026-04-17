#include "runtime/robot_mode_state_machine.hpp"

#include <stdexcept>

namespace ausim {
namespace {

bool ShouldAcceptMotion(const RobotModeConditionContext& context, std::string_view condition) {
  if (condition.empty() || condition == "always") {
    return true;
  }
  if (condition == "motion_active") {
    return context.motion_active;
  }
  if (condition == "motion_inactive") {
    return !context.motion_active;
  }
  throw std::runtime_error("Unsupported robot mode condition: " + std::string(condition));
}

}  // namespace

RobotTopLevelState ParseRobotTopLevelState(std::string_view name) {
  if (name == "SAFE") {
    return RobotTopLevelState::kSafe;
  }
  if (name == "MANUAL_READY") {
    return RobotTopLevelState::kManualReady;
  }
  if (name == "MANUAL_ACTIVE") {
    return RobotTopLevelState::kManualActive;
  }
  if (name == "AUTO") {
    return RobotTopLevelState::kAuto;
  }
  if (name == "FAULT") {
    return RobotTopLevelState::kFault;
  }
  throw std::runtime_error("Unsupported robot top state: " + std::string(name));
}

const char* RobotTopLevelStateName(RobotTopLevelState state) {
  switch (state) {
    case RobotTopLevelState::kSafe:
      return "SAFE";
    case RobotTopLevelState::kManualReady:
      return "MANUAL_READY";
    case RobotTopLevelState::kManualActive:
      return "MANUAL_ACTIVE";
    case RobotTopLevelState::kAuto:
      return "AUTO";
    case RobotTopLevelState::kFault:
      return "FAULT";
  }
  return "SAFE";
}

DiscreteCommandKind ParseDiscreteCommandKind(std::string_view name) {
  if (name.empty() || name == "none") {
    return DiscreteCommandKind::kNone;
  }
  if (name == "reset" || name == "reset_simulation") {
    return DiscreteCommandKind::kResetSimulation;
  }
  if (name == "takeoff") {
    return DiscreteCommandKind::kTakeoff;
  }
  if (name == "mode_next") {
    return DiscreteCommandKind::kModeNext;
  }
  if (name == "estop" || name == "emergency_stop") {
    return DiscreteCommandKind::kEmergencyStop;
  }
  throw std::runtime_error("Unsupported teleop event: " + std::string(name));
}

const char* DiscreteCommandKindName(DiscreteCommandKind kind) {
  switch (kind) {
    case DiscreteCommandKind::kNone:
      return "none";
    case DiscreteCommandKind::kResetSimulation:
      return "reset";
    case DiscreteCommandKind::kTakeoff:
      return "takeoff";
    case DiscreteCommandKind::kModeNext:
      return "mode_next";
    case DiscreteCommandKind::kEmergencyStop:
      return "estop";
  }
  return "none";
}

RobotModeStateMachine::RobotModeStateMachine(RobotModeConfig config) : config_(std::move(config)) {
  for (const RobotModeStateConfig& state_config : config_.states) {
    if (state_config.name.empty()) {
      throw std::runtime_error("teleop.states entries must define a non-empty name");
    }
    states_.emplace(state_config.name,
                    RuntimeState{state_config.name, ParseRobotTopLevelState(state_config.top_state), state_config.accepts_motion});
  }

  enabled_ = !states_.empty() && !config_.initial_state.empty();
  Reset();
}

void RobotModeStateMachine::Reset() {
  snapshot_ = RobotModeSnapshot{};
  if (!enabled_) {
    current_state_name_.clear();
    return;
  }

  const auto state_it = states_.find(config_.initial_state);
  if (state_it == states_.end()) {
    throw std::runtime_error("teleop.initial_state '" + config_.initial_state + "' is not defined in teleop.states");
  }

  current_state_name_ = config_.initial_state;
  UpdateSnapshotForState(state_it->second);
}

bool RobotModeStateMachine::HandleEvent(DiscreteCommandKind event, const RobotModeTransitionCallbacks& callbacks) {
  if (!enabled_ || event == DiscreteCommandKind::kNone) {
    return false;
  }

  for (const RobotModeTransitionConfig& transition : config_.transitions) {
    if (transition.from != current_state_name_) {
      continue;
    }
    if (ParseDiscreteCommandKind(transition.event) != event) {
      continue;
    }
    if (TryApplyTransition(transition, {}, callbacks, event)) {
      return true;
    }
  }
  return false;
}

bool RobotModeStateMachine::UpdateConditions(const RobotModeConditionContext& context, const RobotModeTransitionCallbacks& callbacks) {
  if (!enabled_) {
    return false;
  }

  for (const RobotModeTransitionConfig& transition : config_.transitions) {
    if (transition.from != current_state_name_ || transition.condition.empty()) {
      continue;
    }
    if (TryApplyTransition(transition, context, callbacks, DiscreteCommandKind::kNone)) {
      return true;
    }
  }
  return false;
}

bool RobotModeStateMachine::TryApplyTransition(const RobotModeTransitionConfig& transition, const RobotModeConditionContext& context,
                                               const RobotModeTransitionCallbacks& callbacks, DiscreteCommandKind event) {
  if (event != DiscreteCommandKind::kNone && !transition.condition.empty()) {
    if (!ConditionMatches(transition, context)) {
      return false;
    }
  } else if (event == DiscreteCommandKind::kNone && !ConditionMatches(transition, context)) {
    return false;
  }

  if (!GuardMatches(transition, callbacks) || !ExecuteAction(transition, callbacks)) {
    return false;
  }

  const auto next_state_it = states_.find(transition.to);
  if (next_state_it == states_.end()) {
    throw std::runtime_error("teleop transition target '" + transition.to + "' is not defined in teleop.states");
  }

  current_state_name_ = transition.to;
  UpdateSnapshotForState(next_state_it->second);
  return true;
}

bool RobotModeStateMachine::ConditionMatches(const RobotModeTransitionConfig& transition, const RobotModeConditionContext& context) const {
  return ShouldAcceptMotion(context, transition.condition);
}

bool RobotModeStateMachine::GuardMatches(const RobotModeTransitionConfig& transition, const RobotModeTransitionCallbacks& callbacks) const {
  if (transition.guard.empty()) {
    return true;
  }
  if (!callbacks.evaluate_guard) {
    return false;
  }
  return callbacks.evaluate_guard(transition.guard);
}

bool RobotModeStateMachine::ExecuteAction(const RobotModeTransitionConfig& transition, const RobotModeTransitionCallbacks& callbacks) const {
  if (transition.action.empty()) {
    return true;
  }
  if (!callbacks.execute_action) {
    return false;
  }
  return callbacks.execute_action(transition.action);
}

void RobotModeStateMachine::UpdateSnapshotForState(const RuntimeState& state) {
  snapshot_.top_state = state.top_state;
  snapshot_.sub_state = state.name;
  snapshot_.accepts_motion = state.accepts_motion;
}

}  // namespace ausim
