#pragma once

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <mutex>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "obstacle_config.hpp"

namespace dynamic_obstacle {

class DynamicObstacleManager {
 public:
  DynamicObstacleManager();
  ~DynamicObstacleManager();

  // Initialize with config
  bool Initialize(const ObstacleConfig& config, mjModel* model, mjData* data);

  // Update all obstacles (call after each mj_step)
  void Update();

  // Check if enabled
  bool IsEnabled() const { return enabled_; }

  // Get obstacle count
  int GetObstacleCount() const { return static_cast<int>(obstacles_.size()); }

  // Get debug info
  std::string GetDebugInfo() const;

 private:
  void GenerateObstacles();
  void UpdateSingleObstacle(SingleObstacle& obs, double dt);

  Eigen::Vector3d GenerateRandomPosition();
  Eigen::Vector3d GenerateRandomVelocity(double speed);
  double GenerateRandomSpeed();
  ObstacleShape RandomShapeForMode() const;

  // Add obstacles to MuJoCo model
  bool AddObstacleToModel(const SingleObstacle& obs);

  // Validate that we have a valid model/data
  bool HasValidModelData() const {
    return model_ != nullptr && data_ != nullptr;
  }

  // Configuration
  ObstacleConfig config_;
  bool enabled_ = false;

  // MuJoCo pointers (not owned)
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;

  // Generated obstacles
  std::vector<SingleObstacle> obstacles_;

  // Random number generator (mutable for const methods)
  mutable std::mt19937 rng_;
  mutable std::mutex rng_mutex_;

  // Statistics
  int update_count_ = 0;
  double total_sim_time_ = 0.0;

  // Initialization flag to set initial positions on first update
  bool initialized_ = false;
};

}  // namespace dynamic_obstacle
