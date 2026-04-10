#include "dynamic_obstacle_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "mujoco/mujoco.h"

namespace dyobs = dynamic_obstacle;

namespace {

// Convert shape enum to MuJoCo geom type string
const char* ShapeToGeomType(dyobs::ObstacleShape shape) {
  switch (shape) {
    case dyobs::ObstacleShape::kCylinder: return "cylinder";
    case dyobs::ObstacleShape::kBox: return "box";
    case dyobs::ObstacleShape::kSphere: return "sphere";
    case dyobs::ObstacleShape::kCube: return "box";  // cube uses box type with equal sides
  }
  return "box";
}

}  // namespace

namespace dynamic_obstacle {

DynamicObstacleManager::DynamicObstacleManager()
    : rng_(std::random_device{}()) {}

DynamicObstacleManager::~DynamicObstacleManager() {}

bool DynamicObstacleManager::Initialize(
    const ObstacleConfig& config,
    mjModel* model,
    mjData* data) {
  // Validate config
  std::string error_msg;
  if (!config.IsValid(&error_msg)) {
    std::cerr << "[DynamicObstacleManager] Invalid config: " << error_msg << std::endl;
    return false;
  }

  config_ = config;
  model_ = model;
  data_ = data;

  // Initialize RNG with seed
  {
    std::lock_guard<std::mutex> lock(rng_mutex_);
    rng_.seed(config_.random_seed);
  }

  // Check if max_speed is 0 (static obstacles)
  if (config_.max_speed == 0.0) {
    std::cout << "[DynamicObstacleManager] max_speed=0, generating static obstacles" << std::endl;
  }

  // Generate obstacles
  GenerateObstacles();

  enabled_ = true;
  update_count_ = 0;
  total_sim_time_ = 0.0;
  initialized_ = false;

  std::cout << "[DynamicObstacleManager] Initialized with " << obstacles_.size()
            << " obstacles" << std::endl;

  return true;
}

void DynamicObstacleManager::GenerateObstacles() {
  obstacles_.clear();

  // Scan model for existing geoms with names matching "dynamic_obs_*"
  for (int i = 0; i < model_->ngeom; ++i) {
    const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, i);
    if (geom_name == nullptr) continue;

    // Check if this geom matches our naming pattern
    std::string name(geom_name);
    if (name.find("dynamic_obs_") != 0) continue;

    SingleObstacle obs;
    obs.geom_id = i;
    obs.name = name;

    // Read initial position from model
    obs.initial_position = Eigen::Vector3d(
        model_->geom_pos[i * 3 + 0],
        model_->geom_pos[i * 3 + 1],
        model_->geom_pos[i * 3 + 2]);

    // Read size from model (half-sizes)
    obs.size = Eigen::Vector3d(
        model_->geom_size[i * 3 + 0],
        model_->geom_size[i * 3 + 1],
        model_->geom_size[i * 3 + 2]);

    // Determine shape from geom type
    mjtGeom geom_type = static_cast<mjtGeom>(model_->geom_type[i]);
    switch (geom_type) {
      case mjGEOM_CYLINDER:
        obs.shape = ObstacleShape::kCylinder;
        break;
      case mjGEOM_BOX:
        // Check if it's a cube (equal sides) or box
        if (std::abs(obs.size.x() - obs.size.y()) < 1e-6 &&
            std::abs(obs.size.y() - obs.size.z()) < 1e-6) {
          obs.shape = ObstacleShape::kCube;
        } else {
          obs.shape = ObstacleShape::kBox;
        }
        break;
      case mjGEOM_SPHERE:
        obs.shape = ObstacleShape::kSphere;
        break;
      default:
        obs.shape = ObstacleShape::kBox;  // Default to box for unknown types
    }

    // Generate random speed and velocity
    obs.speed = GenerateRandomSpeed();
    obs.velocity = GenerateRandomVelocity(obs.speed);
    obs.initial_velocity = obs.velocity;

    // Set bounds from config
    obs.bound_x_min = config_.range_x_min;
    obs.bound_x_max = config_.range_x_max;
    obs.bound_y_min = config_.range_y_min;
    obs.bound_y_max = config_.range_y_max;
    obs.bound_z_min = config_.range_z_min;
    obs.bound_z_max = config_.range_z_max;

    obstacles_.push_back(obs);
    std::cout << "[DynamicObstacleManager] Found obstacle: " << obs.name
              << " at (" << obs.initial_position.x() << ", "
              << obs.initial_position.y() << ", " << obs.initial_position.z() << ")"
              << std::endl;
  }

  if (obstacles_.empty()) {
    std::cout << "[DynamicObstacleManager] Warning: No obstacles found in scene!"
              << " Please add geoms with names 'dynamic_obs_0', 'dynamic_obs_1', etc. to your XML."
              << std::endl;
  }
}

Eigen::Vector3d DynamicObstacleManager::GenerateRandomPosition() {
  std::uniform_real_distribution<double> dist_x(
      config_.range_x_min, config_.range_x_max);
  std::uniform_real_distribution<double> dist_y(
      config_.range_y_min, config_.range_y_max);
  std::uniform_real_distribution<double> dist_z(
      config_.range_z_min, config_.range_z_max);

  std::lock_guard<std::mutex> lock(rng_mutex_);

  Eigen::Vector3d pos;
  pos.x() = dist_x(rng_);
  pos.y() = dist_y(rng_);
  pos.z() = dist_z(rng_);

  return pos;
}

Eigen::Vector3d DynamicObstacleManager::GenerateRandomVelocity(double speed) {
  std::uniform_real_distribution<double> dist_angle(0.0, 2.0 * M_PI);

  std::lock_guard<std::mutex> lock(rng_mutex_);

  // Random direction in XY plane (for 2D mode, z velocity is 0)
  double angle = dist_angle(rng_);

  Eigen::Vector3d vel;
  vel.x() = speed * std::cos(angle);
  vel.y() = speed * std::sin(angle);
  vel.z() = 0.0;  // No vertical velocity in 2D mode

  // For 3D mode, add random z velocity
  if (config_.mode == GenerationMode::k3D) {
    std::uniform_real_distribution<double> dist_z_vel(-speed * 0.5, speed * 0.5);
    vel.z() = dist_z_vel(rng_);
  }

  return vel;
}

double DynamicObstacleManager::GenerateRandomSpeed() {
  std::uniform_real_distribution<double> dist_speed(
      config_.min_speed, config_.max_speed);

  std::lock_guard<std::mutex> lock(rng_mutex_);
  return dist_speed(rng_);
}

ObstacleShape DynamicObstacleManager::RandomShapeForMode() const {
  std::uniform_int_distribution<int> dist_shape(0, 1);

  std::lock_guard<std::mutex> lock(rng_mutex_);

  if (config_.mode == GenerationMode::k2D) {
    // In 2D mode, use cylinder (0) or box (1)
    return dist_shape(rng_) == 0 ? ObstacleShape::kCylinder : ObstacleShape::kBox;
  } else {
    // In 3D mode, use sphere (0) or cube (1)
    return dist_shape(rng_) == 0 ? ObstacleShape::kSphere : ObstacleShape::kCube;
  }
}

void DynamicObstacleManager::Update() {
  if (!enabled_ || !HasValidModelData()) {
    return;
  }

  // Get time step from model
  const double dt = model_->opt.timestep;

  // Update all obstacles
  for (auto& obs : obstacles_) {
    UpdateSingleObstacle(obs, dt);
  }

  // Mark as initialized after first update
  initialized_ = true;

  ++update_count_;
  total_sim_time_ += dt;
}

void DynamicObstacleManager::UpdateSingleObstacle(SingleObstacle& obs, double dt) {
  if (!HasValidModelData()) {
    return;
  }

  // Find geom ID by name if not already found
  if (obs.geom_id < 0) {
    obs.geom_id = mj_name2id(model_, mjOBJ_GEOM, obs.name.c_str());
  }

  if (obs.geom_id < 0) {
    // Geom not found, skip
    return;
  }

  // If speed is 0, obstacle is static - use initial position
  if (obs.speed < 1e-6) {
    // Write to model geom_pos - mj_forward() will compute geom_xpos from it
    model_->geom_pos[obs.geom_id * 3 + 0] = obs.initial_position.x();
    model_->geom_pos[obs.geom_id * 3 + 1] = obs.initial_position.y();
    model_->geom_pos[obs.geom_id * 3 + 2] = obs.initial_position.z();
    return;
  }

  // Update position based on velocity
  // Write to model geom_pos - mj_forward() will compute geom_xpos from it
  double new_x = model_->geom_pos[obs.geom_id * 3 + 0] + obs.velocity.x() * dt;
  double new_y = model_->geom_pos[obs.geom_id * 3 + 1] + obs.velocity.y() * dt;
  double new_z = model_->geom_pos[obs.geom_id * 3 + 2] + obs.velocity.z() * dt;

  // Wall collision - bounce back
  bool bounced = false;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();

  // Check X bounds
  double half_size_x = obs.size.x();
  if (new_x - half_size_x < obs.bound_x_min) {
    new_x = obs.bound_x_min + half_size_x;
    obs.velocity.x() = -obs.velocity.x();
    bounced = true;
  } else if (new_x + half_size_x > obs.bound_x_max) {
    new_x = obs.bound_x_max - half_size_x;
    obs.velocity.x() = -obs.velocity.x();
    bounced = true;
  }

  // Check Y bounds
  double half_size_y = obs.size.y();
  if (new_y - half_size_y < obs.bound_y_min) {
    new_y = obs.bound_y_min + half_size_y;
    obs.velocity.y() = -obs.velocity.y();
    bounced = true;
  } else if (new_y + half_size_y > obs.bound_y_max) {
    new_y = obs.bound_y_max - half_size_y;
    obs.velocity.y() = -obs.velocity.y();
    bounced = true;
  }

  // Check Z bounds (for 3D mode)
  if (config_.mode == GenerationMode::k3D) {
    double half_size_z = obs.size.z();
    if (new_z - half_size_z < obs.bound_z_min) {
      new_z = obs.bound_z_min + half_size_z;
      obs.velocity.z() = -obs.velocity.z();
      bounced = true;
    } else if (new_z + half_size_z > obs.bound_z_max) {
      new_z = obs.bound_z_max - half_size_z;
      obs.velocity.z() = -obs.velocity.z();
      bounced = true;
    }
  } else {
    // In 2D mode, keep z at initial position (obstacle sits on ground)
    new_z = obs.initial_position.z();
  }

  // Apply new position to model - mj_forward() will compute geom_xpos from it
  model_->geom_pos[obs.geom_id * 3 + 0] = new_x;
  model_->geom_pos[obs.geom_id * 3 + 1] = new_y;
  model_->geom_pos[obs.geom_id * 3 + 2] = new_z;
}

std::string DynamicObstacleManager::GetDebugInfo() const {
  std::ostringstream oss;
  oss << "DynamicObstacleManager Debug Info:\n";
  oss << "  Enabled: " << (enabled_ ? "yes" : "no") << "\n";
  oss << "  Obstacle count: " << obstacles_.size() << "\n";
  oss << "  Update count: " << update_count_ << "\n";
  oss << "  Total sim time: " << total_sim_time_ << " s\n";
  oss << "  Config:\n";
  oss << "    mode: " << (config_.mode == GenerationMode::k2D ? "2D" : "3D") << "\n";
  oss << "    random_seed: " << config_.random_seed << "\n";
  oss << "    obstacle_count: " << config_.obstacle_count << "\n";
  oss << "    speed range: [" << config_.min_speed << ", " << config_.max_speed << "] m/s\n";
  oss << "    range: [(" << config_.range_x_min << ", " << config_.range_y_min << ", " << config_.range_z_min
      << ") to (" << config_.range_x_max << ", " << config_.range_y_max << ", " << config_.range_z_max << ")]\n";

  if (!obstacles_.empty()) {
    oss << "  Obstacles:\n";
    for (const auto& obs : obstacles_) {
      oss << "    " << obs.name << ": shape=" << static_cast<int>(obs.shape)
          << ", pos=(" << obs.initial_position.x() << ", " << obs.initial_position.y() << ", " << obs.initial_position.z()
          << "), speed=" << obs.speed << " m/s\n";
    }
  }

  return oss.str();
}

}  // namespace dynamic_obstacle
