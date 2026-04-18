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
  elapsed_in_state_ = 0.0;
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

bool RobotModeStateMachine::HandleEvent(std::string_view event_name, const RobotModeTransitionCallbacks& callbacks) {
  if (!enabled_ || event_name.empty()) {
    return false;
  }

  for (const RobotModeTransitionConfig& transition : config_.transitions) {
    if (transition.from != current_state_name_) {
      continue;
    }
    if (transition.event.empty() || transition.event != event_name) {
      continue;
    }
    if (TryApplyTransition(transition, {}, callbacks, TransitionTrigger::kEvent)) {
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
    // Skip transitions that are primarily event- or timeout-driven; condition
    // scanning only picks up pure condition-driven transitions.
    if (!transition.event.empty() || transition.timeout > 0.0) {
      continue;
    }
    if (TryApplyTransition(transition, context, callbacks, TransitionTrigger::kCondition)) {
      return true;
    }
  }
  return false;
}

bool RobotModeStateMachine::Tick(double dt, const RobotModeTransitionCallbacks& callbacks) {
  if (!enabled_ || dt <= 0.0) {
    return false;
  }
  elapsed_in_state_ += dt;

  for (const RobotModeTransitionConfig& transition : config_.transitions) {
    if (transition.from != current_state_name_ || transition.timeout <= 0.0) {
      continue;
    }
    if (elapsed_in_state_ < transition.timeout) {
      continue;
    }
    if (TryApplyTransition(transition, {}, callbacks, TransitionTrigger::kTimeout)) {
      return true;
    }
  }
  return false;
}

bool RobotModeStateMachine::TryApplyTransition(const RobotModeTransitionConfig& transition, const RobotModeConditionContext& context,
                                               const RobotModeTransitionCallbacks& callbacks, TransitionTrigger trigger) {
  // Only condition-driven transitions consult the context. Event-/timeout-driven
  // transitions ignore the `condition` field (load-time warning is emitted if
  // both are set on the same transition).
  if (trigger == TransitionTrigger::kCondition && !ConditionMatches(transition, context)) {
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
  elapsed_in_state_ = 0.0;
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
