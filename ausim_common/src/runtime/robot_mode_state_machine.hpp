#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>

#include "config/quadrotor_config.hpp"
#include "runtime/runtime_types.hpp"

namespace ausim {

struct RobotModeConditionContext {
  bool motion_active = false;
};

struct RobotModeTransitionCallbacks {
  std::function<bool(std::string_view)> evaluate_guard;
  std::function<bool(std::string_view)> execute_action;
};

class RobotModeStateMachine {
 public:
  explicit RobotModeStateMachine(RobotModeConfig config = {});

  bool enabled() const { return enabled_; }
  void Reset();

  const RobotModeSnapshot& Snapshot() const { return snapshot_; }
  bool AcceptsMotion() const { return snapshot_.accepts_motion; }

  bool HandleEvent(DiscreteCommandKind event, const RobotModeTransitionCallbacks& callbacks = {});
  bool UpdateConditions(const RobotModeConditionContext& context, const RobotModeTransitionCallbacks& callbacks = {});

 private:
  struct RuntimeState {
    std::string name;
    RobotTopLevelState top_state = RobotTopLevelState::kSafe;
    bool accepts_motion = false;
  };

  bool TryApplyTransition(const RobotModeTransitionConfig& transition, const RobotModeConditionContext& context,
                          const RobotModeTransitionCallbacks& callbacks, DiscreteCommandKind event);
  bool ConditionMatches(const RobotModeTransitionConfig& transition, const RobotModeConditionContext& context) const;
  bool GuardMatches(const RobotModeTransitionConfig& transition, const RobotModeTransitionCallbacks& callbacks) const;
  bool ExecuteAction(const RobotModeTransitionConfig& transition, const RobotModeTransitionCallbacks& callbacks) const;
  void UpdateSnapshotForState(const RuntimeState& state);

  RobotModeConfig config_;
  std::unordered_map<std::string, RuntimeState> states_;
  std::string current_state_name_;
  RobotModeSnapshot snapshot_;
  bool enabled_ = false;
};

RobotTopLevelState ParseRobotTopLevelState(std::string_view name);
const char* RobotTopLevelStateName(RobotTopLevelState state);
DiscreteCommandKind ParseDiscreteCommandKind(std::string_view name);
const char* DiscreteCommandKindName(DiscreteCommandKind kind);

}  // namespace ausim
