#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>

namespace ausim {

// Registry of action callbacks for the RobotModeStateMachine.
//
// Transitions carry an `action` string; when the state machine applies a
// transition it asks the owning `GoalProvider` (or equivalent) to execute the
// named action via this registry. Keeping the mapping data-driven lets new
// actions be added without touching the state-machine core or per-robot sim
// layers.
template <class Context>
class ModeActionsRegistryT {
 public:
  using Callback = std::function<bool(const Context&)>;

  void Register(std::string name, Callback callback) { actions_[std::move(name)] = std::move(callback); }

  // Returns false if the action name is not registered OR the callback returns
  // false; true otherwise. Empty action names are treated as success (no-op).
  bool Invoke(std::string_view name, const Context& context) const {
    if (name.empty()) {
      return true;
    }
    const auto it = actions_.find(std::string(name));
    if (it == actions_.end() || !it->second) {
      return false;
    }
    return it->second(context);
  }

  bool Has(std::string_view name) const { return actions_.find(std::string(name)) != actions_.end(); }

 private:
  std::unordered_map<std::string, Callback> actions_;
};

}  // namespace ausim
