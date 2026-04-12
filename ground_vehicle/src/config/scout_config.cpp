#include "config/scout_config.hpp"

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace ground_vehicle {
namespace {

template <typename T>
void AssignIfPresent(const YAML::Node& node, const char* key, T* value) {
  if (node && node[key]) {
    *value = node[key].as<T>();
  }
}

fs::path ResolvePath(const fs::path& config_path, const fs::path& maybe_relative_path) {
  if (maybe_relative_path.is_absolute()) {
    return maybe_relative_path;
  }
  return (config_path.parent_path() / maybe_relative_path).lexically_normal();
}

std::optional<fs::path> ResolveRobotConfigPath(const YAML::Node& root, const fs::path& config_path, const fs::path& explicit_robot_path) {
  if (!explicit_robot_path.empty()) {
    return fs::absolute(explicit_robot_path);
  }

  const YAML::Node robot_config_node = root["robot_config"];
  if (robot_config_node && robot_config_node.IsScalar()) {
    return ResolvePath(config_path, robot_config_node.as<std::string>());
  }

  return std::nullopt;
}

void ApplyScoutConfigRoot(const YAML::Node& root, ScoutConfig* config) {
  if (!root || !root.IsMap()) {
    return;
  }

  const YAML::Node bindings_node = root["bindings"];
  if (bindings_node) {
    AssignIfPresent(bindings_node, "freejoint", &config->bindings.freejoint_name);
    AssignIfPresent(bindings_node, "freejoint_name", &config->bindings.freejoint_name);

    const YAML::Node wheel_actuators_node = bindings_node["wheel_actuators"];
    AssignIfPresent(wheel_actuators_node, "front_right", &config->bindings.wheel_actuators.front_right);
    AssignIfPresent(wheel_actuators_node, "front_left", &config->bindings.wheel_actuators.front_left);
    AssignIfPresent(wheel_actuators_node, "rear_left", &config->bindings.wheel_actuators.rear_left);
    AssignIfPresent(wheel_actuators_node, "rear_right", &config->bindings.wheel_actuators.rear_right);
  }

  const YAML::Node drive_node = root["ground_vehicle"];
  AssignIfPresent(drive_node, "wheel_radius", &config->drive.wheel_radius);
  AssignIfPresent(drive_node, "wheel_track", &config->drive.track_width);
  AssignIfPresent(drive_node, "track_width", &config->drive.track_width);
  AssignIfPresent(drive_node, "wheel_base", &config->drive.axle_length);
  AssignIfPresent(drive_node, "wheelbase", &config->drive.axle_length);
  AssignIfPresent(drive_node, "axle_length", &config->drive.axle_length);
  AssignIfPresent(drive_node, "max_linear_speed", &config->drive.max_linear_speed);
  AssignIfPresent(drive_node, "max_angular_speed", &config->drive.max_angular_speed);
  AssignIfPresent(drive_node, "max_wheel_speed", &config->drive.max_wheel_speed);
  if (drive_node) {
    double left_joint_sign = config->drive.front_left_joint_sign;
    double right_joint_sign = config->drive.front_right_joint_sign;
    AssignIfPresent(drive_node, "left_joint_sign", &left_joint_sign);
    AssignIfPresent(drive_node, "right_joint_sign", &right_joint_sign);
    config->drive.front_left_joint_sign = left_joint_sign;
    config->drive.rear_left_joint_sign = left_joint_sign;
    config->drive.front_right_joint_sign = right_joint_sign;
    config->drive.rear_right_joint_sign = right_joint_sign;
  }
  AssignIfPresent(drive_node, "front_right_joint_sign", &config->drive.front_right_joint_sign);
  AssignIfPresent(drive_node, "front_left_joint_sign", &config->drive.front_left_joint_sign);
  AssignIfPresent(drive_node, "rear_left_joint_sign", &config->drive.rear_left_joint_sign);
  AssignIfPresent(drive_node, "rear_right_joint_sign", &config->drive.rear_right_joint_sign);
}

void ApplyScoutSpecificConfig(const fs::path& config_path, const fs::path& explicit_robot_path, ScoutConfig* config) {
  const fs::path absolute_config_path = fs::absolute(config_path);
  if (!fs::exists(absolute_config_path)) {
    throw std::runtime_error("Config file does not exist: " + absolute_config_path.string());
  }

  const YAML::Node root = YAML::LoadFile(absolute_config_path.string());
  ApplyScoutConfigRoot(root, config);

  const std::optional<fs::path> robot_config_path = ResolveRobotConfigPath(root, absolute_config_path, explicit_robot_path);
  if (!robot_config_path.has_value()) {
    return;
  }

  const fs::path absolute_robot_path = fs::absolute(*robot_config_path);
  if (!fs::exists(absolute_robot_path)) {
    throw std::runtime_error("Robot config file does not exist: " + absolute_robot_path.string());
  }
  ApplyScoutConfigRoot(YAML::LoadFile(absolute_robot_path.string()), config);
}

}  // namespace

ScoutConfig LoadScoutConfigFromYaml(const std::string& path) {
  ScoutConfig config;
  config.common = ausim::LoadConfigFromYaml(path);
  ApplyScoutSpecificConfig(path, {}, &config);
  return config;
}

ScoutConfig LoadScoutConfigFromYaml(const std::string& sim_config_path, const std::string& robot_config_path) {
  ScoutConfig config;
  config.common = ausim::LoadConfigFromYaml(sim_config_path, robot_config_path);
  ApplyScoutSpecificConfig(sim_config_path, robot_config_path, &config);
  return config;
}

}  // namespace ground_vehicle
