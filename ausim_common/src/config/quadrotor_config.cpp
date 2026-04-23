#include "config/quadrotor_config.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace ausim {
namespace {

fs::path DefaultSceneXmlPath() { return (fs::path(QUADROTOR_SOURCE_DIR).parent_path() / "assets" / "crazyfile" / "scene.xml").lexically_normal(); }

template <typename T>
void AssignIfPresent(const YAML::Node& node, const char* key, T* value) {
  if (node && node[key]) {
    *value = node[key].as<T>();
  }
}

std::string RequireNonEmptyScalar(const YAML::Node& node, const char* key, const std::string& context) {
  if (!node || !node[key] || !node[key].IsScalar()) {
    throw std::runtime_error(context + " must define scalar '" + std::string(key) + "'");
  }
  const std::string value = node[key].as<std::string>();
  if (value.empty()) {
    throw std::runtime_error(context + " must define non-empty '" + std::string(key) + "'");
  }
  return value;
}

Eigen::Vector3d LoadVector3(const YAML::Node& node, const Eigen::Vector3d& fallback) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    return fallback;
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

fs::path ResolvePath(const fs::path& config_path, const fs::path& maybe_relative_path) {
  if (maybe_relative_path.is_absolute()) {
    return maybe_relative_path;
  }
  return (config_path.parent_path() / maybe_relative_path).lexically_normal();
}

Eigen::Vector3d NormalizeAircraftForwardAxis(const Eigen::Vector3d& axis) {
  const Eigen::Vector3d horizontal_axis(axis.x(), axis.y(), 0.0);
  if (horizontal_axis.norm() < 1e-6) {
    throw std::runtime_error("model.aircraft_forward_axis must have a non-zero horizontal component.");
  }
  if (std::abs(axis.z()) > 1e-6) {
    throw std::runtime_error("model.aircraft_forward_axis must stay in the horizontal plane.");
  }
  return horizontal_axis.normalized();
}

std::string DeriveDepthTopic(const std::string& color_topic) {
  constexpr std::string_view kImageRawSuffix = "/image_raw";
  if (color_topic.size() >= kImageRawSuffix.size() &&
      color_topic.compare(color_topic.size() - kImageRawSuffix.size(), kImageRawSuffix.size(), kImageRawSuffix) == 0) {
    return color_topic.substr(0, color_topic.size() - kImageRawSuffix.size()) + "/depth/image_raw";
  }
  if (color_topic.empty()) {
    return "camera/depth/image_raw";
  }
  return color_topic + "/depth";
}

CameraStreamConfig BuildColorStream(const SensorConfig& sensor) {
  CameraStreamConfig stream;
  stream.name = sensor.name;
  stream.kind = CameraStreamKind::kColor;
  stream.channel_name = sensor.name;
  stream.frame_id = sensor.frame_id;
  stream.topic = sensor.topic;
  stream.rate_hz = sensor.rate_hz;
  return stream;
}

CameraStreamConfig BuildDepthStream(const SensorConfig& sensor) {
  CameraStreamConfig stream;
  stream.name = sensor.name + "_depth";
  stream.kind = CameraStreamKind::kDepth;
  stream.data_type = sensor.depth.data_type;
  stream.channel_name = stream.name;
  stream.frame_id = sensor.depth.frame_id.empty() ? sensor.frame_id : sensor.depth.frame_id;
  stream.topic = sensor.depth.topic.empty() ? DeriveDepthTopic(sensor.topic) : sensor.depth.topic;
  stream.rate_hz = sensor.depth.rate_hz > 0.0 ? sensor.depth.rate_hz : sensor.rate_hz;
  stream.compute_rate_hz = sensor.depth.compute_rate_hz > 0.0 ? sensor.depth.compute_rate_hz : stream.rate_hz;
  stream.worker_threads = sensor.depth.worker_threads;
  return stream;
}

void LoadSensors(const YAML::Node& sensors_node, std::vector<SensorConfig>* sensors) {
  sensors->clear();
  if (!sensors_node || !sensors_node.IsSequence()) {
    return;
  }

  for (const YAML::Node& sensor_node : sensors_node) {
    SensorConfig sensor;
    AssignIfPresent(sensor_node, "name", &sensor.name);
    AssignIfPresent(sensor_node, "type", &sensor.type);
    AssignIfPresent(sensor_node, "enabled", &sensor.enabled);
    AssignIfPresent(sensor_node, "frame_id", &sensor.frame_id);
    AssignIfPresent(sensor_node, "topic", &sensor.topic);
    AssignIfPresent(sensor_node, "rate_hz", &sensor.rate_hz);
    AssignIfPresent(sensor_node, "publish_tf", &sensor.publish_tf);
    const YAML::Node transform_node = sensor_node["transform"];
    if (transform_node && transform_node.IsMap()) {
      const YAML::Node translation_node = transform_node["translation"];
      if (translation_node && translation_node.IsSequence() && translation_node.size() == 3) {
        sensor.transform.translation = {translation_node[0].as<double>(), translation_node[1].as<double>(), translation_node[2].as<double>()};
      }
      const YAML::Node rotation_node = transform_node["rotation"];
      if (rotation_node && rotation_node.IsSequence() && rotation_node.size() == 4) {
        sensor.transform.rotation = {rotation_node[0].as<double>(), rotation_node[1].as<double>(), rotation_node[2].as<double>(), rotation_node[3].as<double>()};
      }
    }
    const YAML::Node depth_node = sensor_node["depth"];
    AssignIfPresent(depth_node, "enabled", &sensor.depth.enabled);
    AssignIfPresent(depth_node, "frame_id", &sensor.depth.frame_id);
    AssignIfPresent(depth_node, "topic", &sensor.depth.topic);
    AssignIfPresent(depth_node, "date_type", &sensor.depth.data_type);
    AssignIfPresent(depth_node, "sensor_data_type", &sensor.depth.data_type);
    AssignIfPresent(depth_node, "sensor_data_types", &sensor.depth.data_type);
    AssignIfPresent(depth_node, "data_type", &sensor.depth.data_type);
    AssignIfPresent(depth_node, "rate_hz", &sensor.depth.rate_hz);
    AssignIfPresent(depth_node, "compute_rate_hz", &sensor.depth.compute_rate_hz);
    AssignIfPresent(depth_node, "worker_threads", &sensor.depth.worker_threads);
    sensors->push_back(std::move(sensor));
  }
}

void LoadRobotModeConfigFromNode(const YAML::Node& mode_node, RobotModeConfig* config) {
  if (config == nullptr || !mode_node || !mode_node.IsMap()) {
    return;
  }

  AssignIfPresent(mode_node, "initial_state", &config->initial_state);

  const YAML::Node states_node = mode_node["states"];
  if (states_node && states_node.IsSequence()) {
    config->states.clear();
    for (const YAML::Node& state_node : states_node) {
      RobotModeStateConfig state;
      AssignIfPresent(state_node, "name", &state.name);
      AssignIfPresent(state_node, "top_state", &state.top_state);
      AssignIfPresent(state_node, "accepts_motion", &state.accepts_motion);
      config->states.push_back(std::move(state));
    }
  }

  const YAML::Node transitions_node = mode_node["transitions"];
  if (transitions_node && transitions_node.IsSequence()) {
    config->transitions.clear();
    for (const YAML::Node& transition_node : transitions_node) {
      RobotModeTransitionConfig transition;
      AssignIfPresent(transition_node, "from", &transition.from);
      AssignIfPresent(transition_node, "to", &transition.to);
      AssignIfPresent(transition_node, "event", &transition.event);
      AssignIfPresent(transition_node, "condition", &transition.condition);
      AssignIfPresent(transition_node, "guard", &transition.guard);
      AssignIfPresent(transition_node, "action", &transition.action);
      AssignIfPresent(transition_node, "timeout", &transition.timeout);

      // event-driven transitions ignore `condition`; warn if both are set so
      // authors don't expect the condition to gate the event.
      if (!transition.event.empty() && !transition.condition.empty()) {
        std::cerr << "[robot_mode_config] warning: transition " << transition.from << " --(" << transition.event
                  << ")--> " << transition.to << " sets both 'event' and 'condition'; the 'condition' will be ignored."
                  << std::endl;
      }
      if (transition.timeout > 0.0 && !transition.event.empty()) {
        std::cerr << "[robot_mode_config] warning: transition " << transition.from << " --> " << transition.to
                  << " sets both 'event' and 'timeout'; only the event trigger will be used." << std::endl;
      }
      config->transitions.push_back(std::move(transition));
    }
  }

  const YAML::Node actions_node = mode_node["actions"];
  if (actions_node && actions_node.IsMap()) {
    const YAML::Node takeoff_node = actions_node["takeoff"];
    AssignIfPresent(takeoff_node, "height", &config->actions.takeoff.height);
    AssignIfPresent(takeoff_node, "climb_rate", &config->actions.takeoff.climb_rate);
    const YAML::Node land_node = actions_node["land"];
    AssignIfPresent(land_node, "descent_rate", &config->actions.land.descent_rate);
  }
}

// Resolves `mode_machine:` / `teleop:` node. Accepts either an inline map or a
// string path to an external YAML file (relative paths resolve against the
// containing config). `teleop:` is accepted as a backward-compatibility alias.
void LoadRobotModeConfig(const YAML::Node& root, const fs::path& config_path, RobotModeConfig* config) {
  if (config == nullptr || !root || !root.IsMap()) {
    return;
  }

  YAML::Node selected;
  if (root["mode_machine"]) {
    selected = root["mode_machine"];
  } else if (root["teleop"]) {
    selected = root["teleop"];
    std::cerr << "[robot_mode_config] warning: 'teleop:' key is deprecated, use 'mode_machine:' instead ("
              << config_path.string() << ")" << std::endl;
  } else {
    return;
  }

  if (selected.IsScalar()) {
    const fs::path external_path = ResolvePath(config_path, selected.as<std::string>());
    if (!fs::exists(external_path)) {
      throw std::runtime_error("mode_machine config file does not exist: " + external_path.string());
    }
    LoadRobotModeConfigFromNode(YAML::LoadFile(external_path.string()), config);
  } else if (selected.IsMap()) {
    LoadRobotModeConfigFromNode(selected, config);
  }
}

void LoadJoyActionServices(const YAML::Node& interfaces_node, std::vector<JoyActionServiceConfig>* action_services) {
  if (action_services == nullptr) {
    return;
  }
  action_services->clear();
  if (!interfaces_node || !interfaces_node["joy_action_services"]) {
    return;
  }

  const YAML::Node services_node = interfaces_node["joy_action_services"];
  if (!services_node.IsSequence()) {
    throw std::runtime_error("interfaces.joy_action_services must be a YAML sequence");
  }

  for (std::size_t index = 0; index < services_node.size(); ++index) {
    const YAML::Node service_node = services_node[index];
    if (!service_node.IsMap()) {
      throw std::runtime_error("interfaces.joy_action_services[" + std::to_string(index) + "] must be a YAML map");
    }

    JoyActionServiceConfig action_service;
    const std::string context = "interfaces.joy_action_services[" + std::to_string(index) + "]";
    action_service.service = RequireNonEmptyScalar(service_node, "service", context);
    action_service.event = RequireNonEmptyScalar(service_node, "event", context);
    action_services->push_back(std::move(action_service));
  }
}

void LoadDynamicObstaclePublishConfig(const fs::path& obstacle_config_path, DynamicObstacleConfig* config) {
  if (config == nullptr) {
    return;
  }

  config->publish = DynamicObstacleConfig::PublishConfig{};
  if (obstacle_config_path.empty() || !fs::exists(obstacle_config_path)) {
    return;
  }

  const YAML::Node root = YAML::LoadFile(obstacle_config_path.string());
  if (!root || !root.IsMap()) {
    throw std::runtime_error("Invalid obstacle config: not a YAML map");
  }

  const YAML::Node publish_node = root["publish"];
  AssignIfPresent(publish_node, "enabled", &config->publish.enabled);
  AssignIfPresent(publish_node, "topic", &config->publish.topic);
  AssignIfPresent(publish_node, "frame_id", &config->publish.frame_id);
  AssignIfPresent(publish_node, "rate_hz", &config->publish.rate_hz);

  if (config->publish.enabled && config->publish.rate_hz <= 0.0) {
    throw std::runtime_error("dynamic_obstacle publish.rate_hz must be positive when publish.enabled=true");
  }
}

void ApplyConfigRoot(const YAML::Node& root, const fs::path& config_path, QuadrotorConfig* config, bool apply_global_simulation_config = true) {
  if (!root || !root.IsMap()) {
    return;
  }

  const YAML::Node robot_node = root["robot"];
  AssignIfPresent(robot_node, "count", &config->robot.count);

  const YAML::Node identity_node = root["identity"];
  AssignIfPresent(identity_node, "vehicle_id", &config->identity.vehicle_id);
  AssignIfPresent(identity_node, "namespace", &config->identity.ros_namespace);
  AssignIfPresent(identity_node, "frame_prefix", &config->identity.frame_prefix);

  const YAML::Node model_node = root["model"];
  if (model_node && model_node["scene_xml"]) {
    config->model.scene_xml = ResolvePath(config_path, model_node["scene_xml"].as<std::string>());
  }
  AssignIfPresent(model_node, "body_name", &config->model.vehicle_body_name);
  if (model_node && model_node["aircraft_forward_axis"]) {
    config->model.aircraft_forward_axis =
        NormalizeAircraftForwardAxis(LoadVector3(model_node["aircraft_forward_axis"], config->model.aircraft_forward_axis));
  }

  if (apply_global_simulation_config) {
    const YAML::Node simulation_node = root["simulation"];
    AssignIfPresent(simulation_node, "duration", &config->simulation.duration);
    AssignIfPresent(simulation_node, "dt", &config->simulation.dt);
    AssignIfPresent(simulation_node, "print_interval", &config->simulation.print_interval);
    AssignIfPresent(simulation_node, "control_mode", &config->simulation.control_mode);
    AssignIfPresent(simulation_node, "example_mode", &config->simulation.example_mode);
    AssignIfPresent(simulation_node, "track_camera_name", &config->simulation.track_camera_name);
    // Backward compatibility for legacy single-file configs.
    if (simulation_node && !simulation_node["track_camera_name"]) {
      AssignIfPresent(model_node, "track_camera_name", &config->simulation.track_camera_name);
    }
    const YAML::Node goal_node = root["goal"];
    if (goal_node) {
      config->hover_goal.position = LoadVector3(goal_node["position"], config->hover_goal.position);
      config->hover_goal.velocity = LoadVector3(goal_node["velocity"], config->hover_goal.velocity);
      config->hover_goal.heading = LoadVector3(goal_node["heading"], config->hover_goal.heading);
    }

    const YAML::Node trajectory_node = root["trajectory"];
    AssignIfPresent(trajectory_node, "wait_time", &config->circle_trajectory.wait_time);
    AssignIfPresent(trajectory_node, "height", &config->circle_trajectory.height);
    AssignIfPresent(trajectory_node, "radius", &config->circle_trajectory.radius);
    AssignIfPresent(trajectory_node, "speed_hz", &config->circle_trajectory.speed_hz);
    AssignIfPresent(trajectory_node, "height_gain", &config->circle_trajectory.height_gain);

    const YAML::Node dynamic_obstacle_node = root["dynamic_obstacle"];
    if (dynamic_obstacle_node) {
      AssignIfPresent(dynamic_obstacle_node, "enabled", &config->dynamic_obstacle.enabled);
      if (dynamic_obstacle_node["config_path"]) {
        config->dynamic_obstacle.config_path = ResolvePath(config_path, dynamic_obstacle_node["config_path"].as<std::string>()).string();
      }
    }
  }

  const YAML::Node vehicle_node = root["vehicle"];
  AssignIfPresent(vehicle_node, "mass", &config->vehicle.mass);
  AssignIfPresent(vehicle_node, "thrust_coefficient", &config->vehicle.Ct);
  AssignIfPresent(vehicle_node, "drag_coefficient", &config->vehicle.Cd);
  AssignIfPresent(vehicle_node, "arm_length", &config->vehicle.arm_length);
  AssignIfPresent(vehicle_node, "max_thrust", &config->vehicle.max_thrust);
  AssignIfPresent(vehicle_node, "max_torque", &config->vehicle.max_torque);
  AssignIfPresent(vehicle_node, "max_speed_krpm", &config->vehicle.max_speed_krpm);

  const YAML::Node controller_node = root["controller"];
  AssignIfPresent(controller_node, "kx", &config->controller.kx);
  AssignIfPresent(controller_node, "kv", &config->controller.kv);
  AssignIfPresent(controller_node, "kR", &config->controller.kR);
  AssignIfPresent(controller_node, "kw", &config->controller.kw);
  AssignIfPresent(controller_node, "rate_hz", &config->controller.rate_hz);
  AssignIfPresent(controller_node, "torque_scale", &config->torque_scale);

  const YAML::Node viewer_node = root["viewer"];
  AssignIfPresent(viewer_node, "enabled", &config->viewer.enabled);
  AssignIfPresent(viewer_node, "fallback_to_headless", &config->viewer.fallback_to_headless);
  AssignIfPresent(viewer_node, "mjui_enabled", &config->viewer.mjui_enabled);
  AssignIfPresent(viewer_node, "vsync", &config->viewer.vsync);
  AssignIfPresent(viewer_node, "show_mode_state_overlay", &config->viewer.show_mode_state_overlay);

  const YAML::Node bindings_node = root["bindings"];
  const YAML::Node actuators_node = bindings_node ? bindings_node["actuators"] : YAML::Node{};
  const YAML::Node state_node = bindings_node ? bindings_node["state"] : YAML::Node{};
  if (actuators_node && actuators_node["motor_names"] && actuators_node["motor_names"].IsSequence() &&
      actuators_node["motor_names"].size() == config->actuators.motor_names.size()) {
    for (std::size_t i = 0; i < config->actuators.motor_names.size(); ++i) {
      config->actuators.motor_names[i] = actuators_node["motor_names"][i].as<std::string>();
    }
  }
  AssignIfPresent(state_node, "gyro_sensor", &config->state.gyro_sensor_name);
  AssignIfPresent(state_node, "accel_sensor", &config->state.accelerometer_sensor_name);
  AssignIfPresent(state_node, "quat_sensor", &config->state.quaternion_sensor_name);

  const YAML::Node ros2_node = root["ros2"];
  AssignIfPresent(ros2_node, "node_name", &config->ros2.node_name);
  AssignIfPresent(ros2_node, "use_sim_time", &config->ros2.use_sim_time);
  AssignIfPresent(ros2_node, "publish_rate_hz", &config->ros2.publish_rate_hz);
  AssignIfPresent(ros2_node, "command_timeout", &config->ros2.command_timeout);
  AssignIfPresent(ros2_node, "publish_tf", &config->ros2.publish_tf);
  AssignIfPresent(ros2_node, "publish_clock", &config->ros2.publish_clock);

  const YAML::Node interfaces_node = root["interfaces"];
  AssignIfPresent(interfaces_node, "cmd_vel_topic", &config->interfaces.cmd_vel_topic);
  AssignIfPresent(interfaces_node, "joy_cmd_vel_topic", &config->interfaces.joy_cmd_vel_topic);
  AssignIfPresent(interfaces_node, "odom_topic", &config->interfaces.odom_topic);
  AssignIfPresent(interfaces_node, "imu_topic", &config->interfaces.imu_topic);
  AssignIfPresent(interfaces_node, "clock_topic", &config->interfaces.clock_topic);
  LoadJoyActionServices(interfaces_node, &config->interfaces.joy_action_services);
  AssignIfPresent(interfaces_node, "robot_mode_topic", &config->interfaces.robot_mode_topic);
  AssignIfPresent(interfaces_node, "robot_mode_structured_topic", &config->interfaces.robot_mode_structured_topic);

  const YAML::Node frames_node = root["frames"];
  AssignIfPresent(frames_node, "odom", &config->frames.odom);
  AssignIfPresent(frames_node, "base", &config->frames.base);
  AssignIfPresent(frames_node, "imu", &config->frames.imu);

  LoadRobotModeConfig(root, config_path, &config->teleop_mode);

  if (root["sensors"]) {
    LoadSensors(root["sensors"], &config->sensors);
  }
}

void ApplyDynamicObstacleEnvOverrides(QuadrotorConfig* config) {
  if (const char* enabled_override = std::getenv("AUSIM_DYNAMIC_OBSTACLE_ENABLED_OVERRIDE");
      enabled_override != nullptr && enabled_override[0] != '\0') {
    const std::string value(enabled_override);
    if (value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON") {
      config->dynamic_obstacle.enabled = true;
    } else if (value == "0" || value == "false" || value == "FALSE" || value == "off" || value == "OFF") {
      config->dynamic_obstacle.enabled = false;
    } else {
      throw std::runtime_error("Invalid AUSIM_DYNAMIC_OBSTACLE_ENABLED_OVERRIDE value: " + value);
    }
  }

  if (const char* path_override = std::getenv("AUSIM_DYNAMIC_OBSTACLE_CONFIG_OVERRIDE"); path_override != nullptr && path_override[0] != '\0') {
    config->dynamic_obstacle.config_path = fs::absolute(path_override).string();
  }
}

std::optional<fs::path> ResolveRobotConfigPath(const YAML::Node& root, const fs::path& config_path, const fs::path& explicit_robot_path,
                                               bool require_robot_config) {
  if (!explicit_robot_path.empty()) {
    return fs::absolute(explicit_robot_path);
  }

  const YAML::Node robot_config_node = root["robot_config"];
  if (robot_config_node && robot_config_node.IsScalar()) {
    return ResolvePath(config_path, robot_config_node.as<std::string>());
  }

  if (require_robot_config) {
    throw std::runtime_error("Simulation config must define 'robot_config' to select the robot model.");
  }
  return std::nullopt;
}

QuadrotorConfig LoadConfigFile(const fs::path& path, const fs::path& explicit_robot_path = {}, bool require_robot_config = false) {
  const fs::path config_path = fs::absolute(path);
  if (!fs::exists(config_path)) {
    throw std::runtime_error("Config file does not exist: " + config_path.string());
  }

  const YAML::Node root = YAML::LoadFile(config_path.string());
  QuadrotorConfig config;
  config.model.scene_xml = DefaultSceneXmlPath();
  ApplyConfigRoot(root, config_path, &config, true);

  const std::optional<fs::path> robot_config_path = ResolveRobotConfigPath(root, config_path, explicit_robot_path, require_robot_config);
  if (robot_config_path.has_value()) {
    const fs::path resolved_robot_config_path = fs::absolute(*robot_config_path);
    if (!fs::exists(resolved_robot_config_path)) {
      throw std::runtime_error("Robot config file does not exist: " + resolved_robot_config_path.string());
    }
    ApplyConfigRoot(YAML::LoadFile(resolved_robot_config_path.string()), resolved_robot_config_path, &config, false);
  }

  ApplyDynamicObstacleEnvOverrides(&config);
  if (!config.dynamic_obstacle.config_path.empty()) {
    LoadDynamicObstaclePublishConfig(fs::absolute(config.dynamic_obstacle.config_path), &config.dynamic_obstacle);
  }

  return config;
}

}  // namespace

QuadrotorConfig LoadConfigFromYaml(const std::string& path) { return LoadConfigFile(path); }

QuadrotorConfig LoadConfigFromYaml(const std::string& sim_config_path, const std::string& robot_config_path) {
  return LoadConfigFile(sim_config_path, robot_config_path, true);
}

std::vector<CameraStreamConfig> BuildCameraStreamConfigs(const std::vector<SensorConfig>& sensors) {
  std::vector<CameraStreamConfig> streams;
  for (const SensorConfig& sensor : sensors) {
    if (!sensor.enabled || sensor.type != "camera") {
      continue;
    }
    streams.push_back(BuildColorStream(sensor));
    if (sensor.depth.enabled) {
      streams.push_back(BuildDepthStream(sensor));
    }
  }
  return streams;
}

}  // namespace ausim
