#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

namespace dynamic_obstacle {

enum class ObstacleShape {
  kCylinder,   // 2D: cylinder (height from 0 to z_range)
  kBox,        // 2D: box (height from 0 to z_range)
  kSphere,     // 3D: sphere
  kCube        // 3D: cube
};

enum class GenerationMode {
  k2D,  // All obstacles start from z=0, use cylinder/box
  k3D   // Random height generation, use sphere/cube
};

struct ObstacleConfig {
  // Basic parameters
  int random_seed = 42;
  bool dynamic = true;
  bool debug = false;
  GenerationMode mode = GenerationMode::k2D;

  // Shape parameters (depending on mode, some will be used)
  double radius = 0.3;        // For cylinder/sphere
  double box_size = 0.5;     // Shared edge length for box/cube

  // Generation range
  double range_x_min = -5.0;
  double range_y_min = -5.0;
  double range_z_min = 0.0;
  double range_x_max = 5.0;
  double range_y_max = 5.0;
  double range_z_max = 2.0;   // For 3D mode, max height

  // Obstacle count
  int obstacle_count = 10;

  // Motion parameters
  double min_speed = 0.0;   // m/s
  double max_speed = 1.0;   // m/s
  bool collision_enabled = false;
  int update_threads = 0;   // 0 = auto, 1 = single-threaded, >1 = total update threads
  int parallel_threshold = 16;  // minimum moving obstacles before enabling worker threads

  // Optional ROS publish settings for dynamic obstacle snapshots.
  bool publish_enabled = false;
  std::string publish_topic = "/dyn_obstacle";
  std::string publish_frame_id = "world";
  double publish_rate_hz = 20.0;

  // Validations
  bool IsValid(std::string* error_msg = nullptr) const;
  int GetUsedShape() const;  // Returns compatible shape based on mode
  bool HasDynamicMotion() const;
};

struct SingleObstacle {
  int geom_id = -1;                    // MuJoCo geom ID
  std::string name;                     // For debugging

  // Shape
  ObstacleShape shape = ObstacleShape::kCylinder;

  // Position and size
  Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d(0.3, 0.3, 1.0);  // half-sizes for MuJoCo

  // Motion state
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // Current velocity
  Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
  double speed = 0.0;

  // Range bounds (for wall bouncing)
  double bound_x_min = -5.0;
  double bound_x_max = 5.0;
  double bound_y_min = -5.0;
  double bound_y_max = 5.0;
  double bound_z_min = 0.0;
  double bound_z_max = 2.0;
};

ObstacleConfig LoadConfigFromYaml(const std::string& path);

}  // namespace dynamic_obstacle
