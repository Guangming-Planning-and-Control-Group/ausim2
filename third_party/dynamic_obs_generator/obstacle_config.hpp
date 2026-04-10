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
  GenerationMode mode = GenerationMode::k2D;

  // Shape parameters (depending on mode, some will be used)
  double radius = 0.3;        // For cylinder/sphere
  double box_length = 0.5;   // For box (square base)
  double box_width = 0.5;    // For box (rectangle base)
  double cube_size = 0.5;    // For cube

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

  // Validations
  bool IsValid(std::string* error_msg = nullptr) const;
  int GetUsedShape() const;  // Returns compatible shape based on mode
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
