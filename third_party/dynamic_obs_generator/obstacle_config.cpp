#include "obstacle_config.hpp"

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
namespace dyobs = dynamic_obstacle;

namespace {

template <typename T>
void AssignIfPresent(const YAML::Node& node, const char* key, T* value) {
  if (node && node[key]) {
    *value = node[key].as<T>();
  }
}

std::string ShapeToString(dyobs::ObstacleShape shape) {
  switch (shape) {
    case dyobs::ObstacleShape::kCylinder: return "cylinder";
    case dyobs::ObstacleShape::kBox: return "box";
    case dyobs::ObstacleShape::kSphere: return "sphere";
    case dyobs::ObstacleShape::kCube: return "cube";
  }
  return "unknown";
}

}  // namespace

namespace dynamic_obstacle {

bool ObstacleConfig::IsValid(std::string* error_msg) const {
  // Check random seed (any integer is valid)
  if (random_seed < 0) {
    if (error_msg) *error_msg = "random_seed must be non-negative";
    return false;
  }

  // Check range
  if (range_x_min >= range_x_max) {
    if (error_msg) *error_msg = "range_x_min must be less than range_x_max";
    return false;
  }
  if (range_y_min >= range_y_max) {
    if (error_msg) *error_msg = "range_y_min must be less than range_y_max";
    return false;
  }
  if (range_z_min < 0) {
    if (error_msg) *error_msg = "range_z_min must be non-negative (floor is at z=0)";
    return false;
  }
  if (range_z_min >= range_z_max) {
    if (error_msg) *error_msg = "range_z_min must be less than range_z_max";
    return false;
  }

  // Check obstacle count
  if (obstacle_count < 0) {
    if (error_msg) *error_msg = "obstacle_count must be non-negative";
    return false;
  }

  // Check speed
  if (min_speed < 0) {
    if (error_msg) *error_msg = "min_speed must be non-negative";
    return false;
  }
  if (max_speed < 0) {
    if (error_msg) *error_msg = "max_speed must be non-negative";
    return false;
  }
  if (min_speed > max_speed) {
    if (error_msg) *error_msg = "min_speed must not exceed max_speed";
    return false;
  }

  // Check size parameters
  if (radius <= 0) {
    if (error_msg) *error_msg = "radius must be positive";
    return false;
  }
  if (box_length <= 0 || box_width <= 0) {
    if (error_msg) *error_msg = "box_length and box_width must be positive";
    return false;
  }
  if (cube_size <= 0) {
    if (error_msg) *error_msg = "cube_size must be positive";
    return false;
  }

  return true;
}

int ObstacleConfig::GetUsedShape() const {
  if (mode == GenerationMode::k2D) {
    // In 2D mode, randomly choose between cylinder and box
    return -1;  // Signal to randomize
  } else {
    // In 3D mode, randomly choose between sphere and cube
    return -2;  // Signal to randomize
  }
}

ObstacleConfig LoadConfigFromYaml(const std::string& path) {
  const fs::path config_path = fs::absolute(path);
  if (!fs::exists(config_path)) {
    throw std::runtime_error("Obstacle config file does not exist: " + config_path.string());
  }

  const YAML::Node root = YAML::LoadFile(config_path.string());
  if (!root || !root.IsMap()) {
    throw std::runtime_error("Invalid obstacle config: not a YAML map");
  }

  ObstacleConfig config;

  // Basic parameters
  AssignIfPresent(root, "random_seed", &config.random_seed);

  // Mode
  if (root["mode"]) {
    std::string mode_str = root["mode"].as<std::string>();
    if (mode_str == "2d" || mode_str == "2D") {
      config.mode = GenerationMode::k2D;
    } else if (mode_str == "3d" || mode_str == "3D") {
      config.mode = GenerationMode::k3D;
    } else {
      throw std::runtime_error("Invalid mode: " + mode_str + ". Use '2d' or '3d'");
    }
  }

  // Size parameters
  AssignIfPresent(root, "radius", &config.radius);
  AssignIfPresent(root, "box_length", &config.box_length);
  AssignIfPresent(root, "box_width", &config.box_width);
  AssignIfPresent(root, "cube_size", &config.cube_size);

  // Generation range
  if (root["range"]) {
    const YAML::Node range = root["range"];
    AssignIfPresent(range, "x_min", &config.range_x_min);
    AssignIfPresent(range, "x_max", &config.range_x_max);
    AssignIfPresent(range, "y_min", &config.range_y_min);
    AssignIfPresent(range, "y_max", &config.range_y_max);
    AssignIfPresent(range, "z_min", &config.range_z_min);
    AssignIfPresent(range, "z_max", &config.range_z_max);
  }

  // Obstacle count
  AssignIfPresent(root, "obstacle_count", &config.obstacle_count);

  // Speed parameters
  AssignIfPresent(root, "min_speed", &config.min_speed);
  AssignIfPresent(root, "max_speed", &config.max_speed);

  // Validate
  std::string error_msg;
  if (!config.IsValid(&error_msg)) {
    throw std::runtime_error("Invalid obstacle config: " + error_msg);
  }

  return config;
}

}  // namespace dynamic_obstacle
