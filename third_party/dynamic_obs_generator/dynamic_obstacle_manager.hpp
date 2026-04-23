#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "obstacle_config.hpp"
#include "runtime/dynamic_obstacles_snapshot.hpp"

namespace dynamic_obstacle {

class DynamicObstacleManager {
 public:
  DynamicObstacleManager();
  ~DynamicObstacleManager();

  // Initialize with config and scan the generated scene for dynamic_obs_* geoms.
  bool Initialize(const ObstacleConfig& config, mjModel* model, mjData* data);

  // Apply the planned obstacle trajectory at the target simulation time.
  bool ApplyTrajectory(double sim_time);

  // False means the scene stays static and no per-step obstacle work is required.
  bool IsEnabled() const { return enabled_; }
  bool RequiresPhysicsRateUpdates() const { return enabled_ && requires_physics_rate_updates_; }

  // Number of dynamic_obs_* geoms discovered in the scene.
  int GetObstacleCount() const { return obstacle_count_; }
  bool FillSnapshot(ausim::DynamicObstaclesSnapshot& out, double sim_time, const std::string& frame_id) const;

  std::string GetDebugInfo() const;

 private:
  struct RuntimeObstacle {
    int geom_id = -1;
    int geom_pos_adr = -1;
    int mocap_body_id = -1;
    int mocap_addr = -1;
    std::string name;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double initial_z = 0.0;
    double half_x = 0.0;
    double half_y = 0.0;
    double half_z = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double velocity_z = 0.0;
    double speed = 0.0;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
    bool collidable = false;
    bool is_mocap = false;
  };

  struct WorkRange {
    std::size_t begin = 0;
    std::size_t end = 0;
  };

  void ScanSceneObstacles();
  int FindObstacleGeomForBody(int body_id, const std::string& body_name) const;
  RuntimeObstacle BuildRuntimeObstacle(int geom_id, int mocap_body_id = -1) const;
  void ApplySingleObstacleTrajectory(RuntimeObstacle& obs, double sim_time, double* geom_pos);
  void ApplyTrajectoryRange(
      std::size_t begin,
      std::size_t end,
      double sim_time,
      double* geom_pos);
  double GenerateRandomSpeed();
  void GenerateRandomVelocity(double speed, double* vx, double* vy, double* vz);

  std::size_t ResolveUpdateThreadCount() const;
  void ConfigureWorkerPool();
  void StartWorkerPool(std::size_t thread_count);
  void StopWorkerPool();
  void WorkerLoop(std::size_t worker_index);

  bool HasValidModelData() const {
    return model_ != nullptr && data_ != nullptr;
  }

  ObstacleConfig config_;
  bool enabled_ = false;
  bool requires_physics_rate_updates_ = false;

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;

  // Only moving obstacles are kept here. Static obstacles stay in the XML and
  // need no runtime work.
  std::vector<RuntimeObstacle> scene_obstacles_;
  std::vector<RuntimeObstacle> moving_obstacles_;
  int obstacle_count_ = 0;

  std::mt19937 rng_;

  bool use_parallel_update_ = false;
  std::size_t total_update_threads_ = 1;
  WorkRange main_work_range_{};
  std::vector<WorkRange> worker_ranges_;
  std::vector<std::thread> worker_threads_;

  mutable std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  std::condition_variable worker_done_cv_;
  std::size_t active_workers_ = 0;
  std::size_t work_generation_ = 0;
  bool stop_workers_ = false;
  double pending_sim_time_ = 0.0;
  double* pending_geom_pos_ = nullptr;

  int update_count_ = 0;
  double total_sim_time_ = 0.0;
  double last_applied_sim_time_ = -1.0;
};

}  // namespace dynamic_obstacle
