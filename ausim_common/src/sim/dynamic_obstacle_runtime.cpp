#include "sim/dynamic_obstacle_runtime.hpp"

#include <exception>
#include <iostream>
#include <utility>

#include "obstacle_config.hpp"

namespace ausim {

void DynamicObstacleRuntime::Initialize(const DynamicObstacleConfig& config, mjModel* model, mjData* data, const std::string& log_prefix) {
  Clear();
  model_ = model;
  data_ = data;

  if (!config.enabled) {
    return;
  }
  if (model_ == nullptr || data_ == nullptr) {
    std::cerr << log_prefix << " Cannot initialize dynamic obstacle manager with null model/data.\n";
    return;
  }
  if (config.config_path.empty()) {
    std::cerr << log_prefix << " Warning: dynamic_obstacle.enabled=true but config_path is not set.\n";
    std::cerr << log_prefix << " Warning: Please set dynamic_obstacle.config_path via config or env override.\n";
    return;
  }

  try {
    auto manager = std::make_unique<dynamic_obstacle::DynamicObstacleManager>();
    const auto obstacle_config = dynamic_obstacle::LoadConfigFromYaml(config.config_path);
    if (!manager->Initialize(obstacle_config, model_, data_)) {
      std::cerr << log_prefix << " Failed to initialize dynamic obstacle manager.\n";
      return;
    }
    if (!manager->IsEnabled()) {
      std::cout << log_prefix << " Dynamic obstacles resolved to a static scene. Runtime obstacle updates are disabled.\n";
      return;
    }

    manager_ = std::move(manager);
    std::cout << log_prefix << " Dynamic obstacle manager initialized successfully.\n";
    if (obstacle_config.debug) {
      std::cout << manager_->GetDebugInfo() << '\n';
    }
    ResetToCurrentTime();
  } catch (const std::exception& error) {
    manager_.reset();
    std::cerr << log_prefix << " Exception initializing dynamic obstacle manager: " << error.what() << '\n';
  }
}

bool DynamicObstacleRuntime::PrepareForStep(
    double next_sim_time,
    bool has_renderable_depth_stream,
    bool cadence_due) {
  if (manager_ == nullptr || model_ == nullptr || data_ == nullptr) {
    return false;
  }

  const bool update_due =
      manager_->RequiresPhysicsRateUpdates() || !has_renderable_depth_stream || cadence_due;
  if (!update_due) {
    return false;
  }

  return manager_->ApplyTrajectory(next_sim_time);
}

bool DynamicObstacleRuntime::ResetToCurrentTime() {
  if (manager_ == nullptr || data_ == nullptr) {
    return false;
  }
  return manager_->ApplyTrajectory(data_->time);
}

void DynamicObstacleRuntime::Clear() {
  manager_.reset();
  model_ = nullptr;
  data_ = nullptr;
}

bool DynamicObstacleRuntime::RequiresPhysicsRateUpdates() const {
  return manager_ != nullptr && manager_->RequiresPhysicsRateUpdates();
}

}  // namespace ausim
