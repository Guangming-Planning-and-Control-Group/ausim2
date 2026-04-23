#include "dynamic_obstacle_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_set>

namespace {

constexpr double kStaticSpeedEpsilon = 1e-6;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::size_t kPreferredObstaclesPerThread = 8;
constexpr const char* kDynamicObstaclePrefix = "dynamic_obs_";

double ClampToBounds(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

double ComputeReflectedCoordinate(
    double initial_position,
    double velocity,
    double min_value,
    double max_value,
    double sim_time) {
  const double clamped_initial = ClampToBounds(initial_position, min_value, max_value);
  const double span = max_value - min_value;
  if (std::abs(velocity) <= kStaticSpeedEpsilon || span <= kStaticSpeedEpsilon) {
    return clamped_initial;
  }

  const double period = 2.0 * span;
  double offset = (clamped_initial - min_value) + velocity * sim_time;
  offset = std::fmod(offset, period);
  if (offset < 0.0) {
    offset += period;
  }

  if (offset <= span) {
    return min_value + offset;
  }
  return max_value - (offset - span);
}

bool HasDynamicObstaclePrefix(const std::string& name) {
  return name.rfind(kDynamicObstaclePrefix, 0) == 0;
}

}  // namespace

namespace dynamic_obstacle {

DynamicObstacleManager::DynamicObstacleManager()
    : rng_(std::random_device{}()) {}

DynamicObstacleManager::~DynamicObstacleManager() {
  StopWorkerPool();
}

bool DynamicObstacleManager::Initialize(
    const ObstacleConfig& config,
    mjModel* model,
    mjData* data) {
  std::string error_msg;
  if (!config.IsValid(&error_msg)) {
    std::cerr << "[DynamicObstacleManager] Invalid config: " << error_msg << std::endl;
    return false;
  }
  if (model == nullptr || data == nullptr) {
    std::cerr << "[DynamicObstacleManager] Cannot initialize with null model/data." << std::endl;
    return false;
  }

  StopWorkerPool();

  config_ = config;
  model_ = model;
  data_ = data;
  scene_obstacles_.clear();
  moving_obstacles_.clear();
  obstacle_count_ = 0;
  enabled_ = false;
  requires_physics_rate_updates_ = false;

  if (config_.random_seed == 0) {
    rng_.seed(std::random_device{}());
  } else {
    rng_.seed(config_.random_seed);
  }

  ScanSceneObstacles();
  ConfigureWorkerPool();

  enabled_ = !moving_obstacles_.empty();
  update_count_ = 0;
  total_sim_time_ = 0.0;
  last_applied_sim_time_ = -1.0;

  if (obstacle_count_ == 0) {
    std::cout << "[DynamicObstacleManager] No dynamic_obs_* geoms found in the generated scene."
              << std::endl;
  } else if (!config_.dynamic) {
    std::cout << "[DynamicObstacleManager] Scene contains " << obstacle_count_
              << " obstacles, but obstacle_config.dynamic=false. "
              << "Runtime obstacle updates disabled." << std::endl;
  } else if (!enabled_) {
    std::cout << "[DynamicObstacleManager] Scene contains " << obstacle_count_
              << " obstacles, but all sampled speeds are zero. "
              << "Runtime obstacle updates disabled." << std::endl;
  } else {
    std::cout << "[DynamicObstacleManager] Initialized with " << obstacle_count_
              << " obstacles, " << moving_obstacles_.size() << " of them moving, using "
              << total_update_threads_ << " update thread(s)"
              << (requires_physics_rate_updates_ ? " at physics cadence."
                                                 : " in trajectory-player mode with shared runtime cadence.")
              << std::endl;
  }

  return true;
}

void DynamicObstacleManager::ScanSceneObstacles() {
  scene_obstacles_.clear();
  moving_obstacles_.clear();
  obstacle_count_ = 0;
  std::unordered_set<int> handled_geom_ids;

  for (int body_id = 0; body_id < model_->nbody; ++body_id) {
    const char* body_name = mj_id2name(model_, mjOBJ_BODY, body_id);
    if (body_name == nullptr) {
      continue;
    }

    const std::string name(body_name);
    if (!HasDynamicObstaclePrefix(name) || model_->body_mocapid[body_id] < 0) {
      continue;
    }

    const int geom_id = FindObstacleGeomForBody(body_id, name);
    if (geom_id < 0) {
      if (config_.debug) {
        std::cerr << "[DynamicObstacleManager] Skipping mocap obstacle body '" << name
                  << "' because no matching child geom was found." << std::endl;
      }
      continue;
    }

    handled_geom_ids.insert(geom_id);
    ++obstacle_count_;
    RuntimeObstacle obs = BuildRuntimeObstacle(geom_id, body_id);
    scene_obstacles_.push_back(obs);
    if (config_.debug) {
      std::cout << "[DynamicObstacleManager] Found obstacle: " << obs.name
                << " at (" << obs.initial_x << ", " << obs.initial_y << ", "
                << obs.initial_z << "), collidable="
                << (obs.collidable ? "yes" : "no")
                << ", path=" << (obs.is_mocap ? "mocap" : "direct") << std::endl;
    }
    if (obs.collidable && !obs.is_mocap) {
      std::cerr << "[DynamicObstacleManager] Warning: collidable dynamic_obs must use a mocap body. "
                   "Teleporting a collidable geom can destabilize contact solving."
                << " Obstacle: " << obs.name << std::endl;
    }

    if (!config_.HasDynamicMotion()) {
      continue;
    }

    obs.speed = GenerateRandomSpeed();
    if (obs.speed <= kStaticSpeedEpsilon) {
      continue;
    }

    GenerateRandomVelocity(
        obs.speed,
        &obs.velocity_x,
        &obs.velocity_y,
        &obs.velocity_z);
    requires_physics_rate_updates_ =
        requires_physics_rate_updates_ || (obs.collidable && !obs.is_mocap);
    moving_obstacles_.push_back(obs);
  }

  for (int geom_id = 0; geom_id < model_->ngeom; ++geom_id) {
    if (handled_geom_ids.count(geom_id) != 0) {
      continue;
    }

    const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, geom_id);
    if (geom_name == nullptr) {
      continue;
    }

    const std::string name(geom_name);
    if (!HasDynamicObstaclePrefix(name)) {
      continue;
    }

    ++obstacle_count_;
    RuntimeObstacle obs = BuildRuntimeObstacle(geom_id);
    scene_obstacles_.push_back(obs);
    if (config_.debug) {
      std::cout << "[DynamicObstacleManager] Found obstacle: " << obs.name
                << " at (" << obs.initial_x << ", " << obs.initial_y << ", "
                << obs.initial_z << "), collidable="
                << (obs.collidable ? "yes" : "no")
                << ", path=" << (obs.is_mocap ? "mocap" : "direct") << std::endl;
    }
    if (obs.collidable && !obs.is_mocap) {
      std::cerr << "[DynamicObstacleManager] Warning: collidable dynamic_obs must use a mocap body. "
                   "Teleporting a collidable geom can destabilize contact solving."
                << " Obstacle: " << obs.name << std::endl;
    }

    if (!config_.HasDynamicMotion()) {
      continue;
    }

    obs.speed = GenerateRandomSpeed();
    if (obs.speed <= kStaticSpeedEpsilon) {
      continue;
    }

    GenerateRandomVelocity(
        obs.speed,
        &obs.velocity_x,
        &obs.velocity_y,
        &obs.velocity_z);
    requires_physics_rate_updates_ =
        requires_physics_rate_updates_ || (obs.collidable && !obs.is_mocap);
    moving_obstacles_.push_back(obs);
  }
}

int DynamicObstacleManager::FindObstacleGeomForBody(int body_id, const std::string& body_name) const {
  int fallback_geom_id = -1;
  const std::string preferred_geom_name = body_name + "_geom";
  for (int geom_id = 0; geom_id < model_->ngeom; ++geom_id) {
    if (model_->geom_bodyid[geom_id] != body_id) {
      continue;
    }

    if (fallback_geom_id < 0) {
      fallback_geom_id = geom_id;
    }

    const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, geom_id);
    if (geom_name == nullptr) {
      continue;
    }
    if (preferred_geom_name == geom_name || body_name == geom_name) {
      return geom_id;
    }
  }

  return fallback_geom_id;
}

DynamicObstacleManager::RuntimeObstacle DynamicObstacleManager::BuildRuntimeObstacle(
    int geom_id,
    int mocap_body_id) const {
  RuntimeObstacle obs;
  obs.geom_id = geom_id;
  obs.geom_pos_adr = geom_id * 3;

  const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, geom_id);
  if (geom_name != nullptr) {
    obs.name = geom_name;
  }
  obs.collidable =
      model_->geom_contype[geom_id] != 0 || model_->geom_conaffinity[geom_id] != 0;

  if (mocap_body_id >= 0) {
    obs.is_mocap = true;
    obs.mocap_body_id = mocap_body_id;
    if (const char* body_name = mj_id2name(model_, mjOBJ_BODY, mocap_body_id);
        body_name != nullptr) {
      obs.name = body_name;
    }

    const int mocap_id = model_->body_mocapid[mocap_body_id];
    if (mocap_id >= 0) {
      obs.mocap_addr = mocap_id * 3;
    }

    const double* initial_pos = nullptr;
    if (data_ != nullptr && obs.mocap_addr >= 0) {
      initial_pos = data_->mocap_pos + obs.mocap_addr;
    } else {
      initial_pos = model_->body_pos + mocap_body_id * 3;
    }
    obs.initial_x = initial_pos[0];
    obs.initial_y = initial_pos[1];
    obs.initial_z = initial_pos[2];
  } else {
    obs.initial_x = model_->geom_pos[obs.geom_pos_adr + 0];
    obs.initial_y = model_->geom_pos[obs.geom_pos_adr + 1];
    obs.initial_z = model_->geom_pos[obs.geom_pos_adr + 2];
  }

  const int size_adr = geom_id * 3;
  const mjtGeom geom_type = static_cast<mjtGeom>(model_->geom_type[geom_id]);
  switch (geom_type) {
    case mjGEOM_CYLINDER:
      obs.half_x = model_->geom_size[size_adr + 0];
      obs.half_y = model_->geom_size[size_adr + 0];
      obs.half_z = model_->geom_size[size_adr + 1];
      break;
    case mjGEOM_SPHERE:
      obs.half_x = model_->geom_size[size_adr + 0];
      obs.half_y = model_->geom_size[size_adr + 0];
      obs.half_z = model_->geom_size[size_adr + 0];
      break;
    case mjGEOM_BOX:
    default:
      obs.half_x = model_->geom_size[size_adr + 0];
      obs.half_y = model_->geom_size[size_adr + 1];
      obs.half_z = model_->geom_size[size_adr + 2];
      break;
  }

  obs.min_x = config_.range_x_min + obs.half_x;
  obs.max_x = config_.range_x_max - obs.half_x;
  obs.min_y = config_.range_y_min + obs.half_y;
  obs.max_y = config_.range_y_max - obs.half_y;
  obs.min_z = config_.range_z_min + obs.half_z;
  obs.max_z = config_.range_z_max - obs.half_z;
  return obs;
}

double DynamicObstacleManager::GenerateRandomSpeed() {
  std::uniform_real_distribution<double> dist_speed(config_.min_speed, config_.max_speed);
  return dist_speed(rng_);
}

void DynamicObstacleManager::GenerateRandomVelocity(double speed, double* vx, double* vy, double* vz) {
  std::uniform_real_distribution<double> dist_angle(0.0, kTwoPi);
  const double angle = dist_angle(rng_);

  *vx = speed * std::cos(angle);
  *vy = speed * std::sin(angle);
  *vz = 0.0;

  if (config_.mode == GenerationMode::k3D) {
    std::uniform_real_distribution<double> dist_z_velocity(-speed * 0.5, speed * 0.5);
    *vz = dist_z_velocity(rng_);
  }
}

std::size_t DynamicObstacleManager::ResolveUpdateThreadCount() const {
  const std::size_t moving_count = moving_obstacles_.size();
  if (moving_count < 2) {
    return 1;
  }

  if (config_.parallel_threshold > 0 &&
      moving_count < static_cast<std::size_t>(config_.parallel_threshold)) {
    return 1;
  }

  std::size_t thread_count = 1;
  if (config_.update_threads > 0) {
    thread_count = static_cast<std::size_t>(config_.update_threads);
  } else {
    const std::size_t hardware_threads = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::thread::hardware_concurrency()));
    const std::size_t size_based_threads =
        std::max<std::size_t>(2, (moving_count + kPreferredObstaclesPerThread - 1) /
                                     kPreferredObstaclesPerThread);
    thread_count = std::min(hardware_threads, size_based_threads);
    if (thread_count == 0) {
      thread_count = 1;
    }
  }

  thread_count = std::max<std::size_t>(1, thread_count);
  return std::min(thread_count, moving_count);
}

void DynamicObstacleManager::ConfigureWorkerPool() {
  StopWorkerPool();

  total_update_threads_ = ResolveUpdateThreadCount();
  main_work_range_ = WorkRange{0, moving_obstacles_.size()};
  use_parallel_update_ = total_update_threads_ > 1;

  if (!use_parallel_update_) {
    return;
  }

  const std::size_t moving_count = moving_obstacles_.size();
  const std::size_t base_chunk = moving_count / total_update_threads_;
  const std::size_t remainder = moving_count % total_update_threads_;

  std::size_t begin = 0;
  std::vector<WorkRange> all_ranges;
  all_ranges.reserve(total_update_threads_);
  for (std::size_t index = 0; index < total_update_threads_; ++index) {
    const std::size_t count = base_chunk + (index < remainder ? 1 : 0);
    all_ranges.push_back(WorkRange{begin, begin + count});
    begin += count;
  }

  main_work_range_ = all_ranges.front();
  worker_ranges_.assign(all_ranges.begin() + 1, all_ranges.end());
  StartWorkerPool(worker_ranges_.size());
}

void DynamicObstacleManager::StartWorkerPool(std::size_t thread_count) {
  if (thread_count == 0) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    stop_workers_ = false;
    active_workers_ = 0;
    work_generation_ = 0;
    pending_sim_time_ = 0.0;
    pending_geom_pos_ = nullptr;
  }

  worker_threads_.reserve(thread_count);
  for (std::size_t worker_index = 0; worker_index < thread_count; ++worker_index) {
    worker_threads_.emplace_back(&DynamicObstacleManager::WorkerLoop, this, worker_index);
  }
}

void DynamicObstacleManager::StopWorkerPool() {
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    stop_workers_ = true;
    ++work_generation_;
  }
  worker_cv_.notify_all();

  for (std::thread& worker : worker_threads_) {
    if (worker.joinable()) {
      worker.join();
    }
  }

  worker_threads_.clear();
  worker_ranges_.clear();
  main_work_range_ = WorkRange{0, moving_obstacles_.size()};
  use_parallel_update_ = false;
  total_update_threads_ = 1;

  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    stop_workers_ = false;
    active_workers_ = 0;
    pending_sim_time_ = 0.0;
    pending_geom_pos_ = nullptr;
  }
}

void DynamicObstacleManager::WorkerLoop(std::size_t worker_index) {
  std::size_t local_generation = 0;

  while (true) {
    WorkRange range;
    double sim_time = 0.0;
    double* geom_pos = nullptr;

    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_cv_.wait(lock, [this, &local_generation] {
        return stop_workers_ || work_generation_ != local_generation;
      });

      if (stop_workers_) {
        return;
      }

      local_generation = work_generation_;
      range = worker_ranges_[worker_index];
      sim_time = pending_sim_time_;
      geom_pos = pending_geom_pos_;
    }

    ApplyTrajectoryRange(range.begin, range.end, sim_time, geom_pos);

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (active_workers_ > 0) {
        --active_workers_;
        if (active_workers_ == 0) {
          worker_done_cv_.notify_one();
        }
      }
    }
  }
}

bool DynamicObstacleManager::ApplyTrajectory(double sim_time) {
  if (!enabled_ || !HasValidModelData()) {
    return false;
  }
  if (sim_time < 0.0) {
    return false;
  }
  if (std::abs(sim_time - last_applied_sim_time_) <= kStaticSpeedEpsilon) {
    return false;
  }

  double* geom_pos = model_->geom_pos;

  if (!use_parallel_update_ || worker_threads_.empty()) {
    ApplyTrajectoryRange(0, moving_obstacles_.size(), sim_time, geom_pos);
  } else {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      pending_sim_time_ = sim_time;
      pending_geom_pos_ = geom_pos;
      active_workers_ = worker_threads_.size();
      ++work_generation_;
    }
    worker_cv_.notify_all();

    ApplyTrajectoryRange(main_work_range_.begin, main_work_range_.end, sim_time, geom_pos);

    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_done_cv_.wait(lock, [this] { return active_workers_ == 0; });
  }

  ++update_count_;
  total_sim_time_ = sim_time;
  last_applied_sim_time_ = sim_time;
  return true;
}

bool DynamicObstacleManager::FillSnapshot(ausim::DynamicObstaclesSnapshot& out, double sim_time, const std::string& frame_id) const {
  if (!HasValidModelData()) {
    return false;
  }

  out = ausim::DynamicObstaclesSnapshot{};
  out.sim_time = sim_time;
  out.frame_id = frame_id;
  out.entries.reserve(scene_obstacles_.size());
  for (const RuntimeObstacle& obs : scene_obstacles_) {
    ausim::DynamicObstacleEntry entry;
    entry.name = obs.name;
    entry.pos[0] = model_->geom_pos[obs.geom_pos_adr + 0];
    entry.pos[1] = model_->geom_pos[obs.geom_pos_adr + 1];
    entry.pos[2] = model_->geom_pos[obs.geom_pos_adr + 2];
    entry.quat[0] = 1.0;
    entry.quat[1] = 0.0;
    entry.quat[2] = 0.0;
    entry.quat[3] = 0.0;
    entry.size[0] = obs.half_x * 2.0;
    entry.size[1] = obs.half_y * 2.0;
    entry.size[2] = obs.half_z * 2.0;
    out.entries.push_back(std::move(entry));
  }

  return true;
}

void DynamicObstacleManager::ApplyTrajectoryRange(
    std::size_t begin,
    std::size_t end,
    double sim_time,
    double* geom_pos) {
  for (std::size_t index = begin; index < end; ++index) {
    ApplySingleObstacleTrajectory(moving_obstacles_[index], sim_time, geom_pos);
  }
}

void DynamicObstacleManager::ApplySingleObstacleTrajectory(
    RuntimeObstacle& obs,
    double sim_time,
    double* geom_pos) {
  const double new_x = ComputeReflectedCoordinate(
      obs.initial_x, obs.velocity_x, obs.min_x, obs.max_x, sim_time);
  const double new_y = ComputeReflectedCoordinate(
      obs.initial_y, obs.velocity_y, obs.min_y, obs.max_y, sim_time);

  double new_z = obs.initial_z;
  if (config_.mode == GenerationMode::k3D) {
    new_z = ComputeReflectedCoordinate(
        obs.initial_z, obs.velocity_z, obs.min_z, obs.max_z, sim_time);
  }

  if (obs.is_mocap) {
    if (obs.mocap_addr >= 0 && data_ != nullptr) {
      data_->mocap_pos[obs.mocap_addr + 0] = new_x;
      data_->mocap_pos[obs.mocap_addr + 1] = new_y;
      data_->mocap_pos[obs.mocap_addr + 2] = new_z;
    }
    return;
  }

  geom_pos[obs.geom_pos_adr + 0] = new_x;
  geom_pos[obs.geom_pos_adr + 1] = new_y;
  geom_pos[obs.geom_pos_adr + 2] = new_z;
}

std::string DynamicObstacleManager::GetDebugInfo() const {
  std::ostringstream oss;
  oss << "DynamicObstacleManager Debug Info:\n";
  oss << "  Enabled: " << (enabled_ ? "yes" : "no") << "\n";
  oss << "  Obstacle count: " << obstacle_count_ << "\n";
  oss << "  Moving obstacle count: " << moving_obstacles_.size() << "\n";
  oss << "  Update count: " << update_count_ << "\n";
  oss << "  Total sim time: " << total_sim_time_ << " s\n";
  oss << "  Config:\n";
  oss << "    dynamic: " << (config_.dynamic ? "true" : "false") << "\n";
  oss << "    debug: " << (config_.debug ? "true" : "false") << "\n";
  oss << "    mode: " << (config_.mode == GenerationMode::k2D ? "2D" : "3D") << "\n";
  oss << "    random_seed: " << config_.random_seed << "\n";
  oss << "    obstacle_count: " << config_.obstacle_count << "\n";
  oss << "    speed range: [" << config_.min_speed << ", " << config_.max_speed << "] m/s\n";
  oss << "    collision_enabled: " << (config_.collision_enabled ? "true" : "false") << "\n";
  oss << "    update_threads: " << config_.update_threads << "\n";
  oss << "    parallel_threshold: " << config_.parallel_threshold << "\n";
  oss << "    active_update_threads: " << total_update_threads_ << "\n";
  oss << "    parallel_update: " << (use_parallel_update_ ? "yes" : "no") << "\n";
  oss << "    update_cadence: "
      << (requires_physics_rate_updates_
              ? "physics-step"
              : "shared runtime cadence (depth-driven when available, otherwise step-driven)")
      << "\n";
  oss << "    range: [(" << config_.range_x_min << ", " << config_.range_y_min << ", "
      << config_.range_z_min << ") to (" << config_.range_x_max << ", "
      << config_.range_y_max << ", " << config_.range_z_max << ")]\n";

  if (config_.debug && !moving_obstacles_.empty()) {
    oss << "  Moving Obstacles:\n";
    for (const RuntimeObstacle& obs : moving_obstacles_) {
      oss << "    " << obs.name
          << ": pos=(" << obs.initial_x << ", " << obs.initial_y << ", " << obs.initial_z << ")"
          << ", speed=" << obs.speed << " m/s"
          << ", velocity=(" << obs.velocity_x << ", " << obs.velocity_y << ", "
          << obs.velocity_z << ")"
          << ", collidable=" << (obs.collidable ? "yes" : "no")
          << ", path=" << (obs.is_mocap ? "mocap" : "direct") << "\n";
    }
  } else if (!moving_obstacles_.empty()) {
    oss << "  Moving obstacle details hidden (set debug=true to print each obstacle).\n";
  }

  return oss.str();
}

}  // namespace dynamic_obstacle
