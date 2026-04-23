#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "config/quadrotor_config.hpp"
#include "dynamic_obstacle_manager.hpp"
#include "obstacle_config.hpp"
#include "runtime/dynamic_obstacles_snapshot.hpp"

namespace ausim {

class DynamicObstacleRuntime {
 public:
  void Initialize(const DynamicObstacleConfig& config, mjModel* model, mjData* data, const std::string& log_prefix);
  bool PrepareForStep(double next_sim_time, bool has_renderable_depth_stream, bool cadence_due);
  bool ResetToCurrentTime();
  void Clear();
  bool RequiresPhysicsRateUpdates() const;
  bool BuildSnapshot(DynamicObstaclesSnapshot& out) const;
  bool PublishEnabled() const;
  double PublishRateHz() const;

 private:
  std::unique_ptr<dynamic_obstacle::DynamicObstacleManager> manager_;
  dynamic_obstacle::ObstacleConfig obstacle_config_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
};

}  // namespace ausim
