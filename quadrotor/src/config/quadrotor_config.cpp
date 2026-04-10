#include "config/quadrotor_config.hpp"

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace quadrotor {
namespace {

fs::path DefaultSceneXmlPath() {
  return (fs::path(QUADROTOR_SOURCE_DIR).parent_path() / "assets" / "crazyfile" / "scene.xml")
      .lexically_normal();
}

template <typename T>
void AssignIfPresent(const YAML::Node& node, const char* key, T* value) {
  if (node && node[key]) {
    *value = node[key].as<T>();
  }
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
    throw std::runtime_error(
        "model.aircraft_forward_axis must have a non-zero horizontal component.");
  }
  if (std::abs(axis.z()) > 1e-6) {
    throw std::runtime_error(
        "model.aircraft_forward_axis must stay in the horizontal plane.");
  }
  return horizontal_axis.normalized();
}

std::string DeriveDepthTopic(const std::string& color_topic) {
  constexpr std::string_view kImageRawSuffix = "/image_raw";
  if (color_topic.size() >= kImageRawSuffix.size() &&
      color_topic.compare(
          color_topic.size() - kImageRawSuffix.size(),
          kImageRawSuffix.size(),
          kImageRawSuffix) == 0) {
    return color_topic.substr(0, color_topic.size() - kImageRawSuffix.size()) +
           "/depth/image_raw";
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
  stream.channel_name = sensor.source_name;
  stream.camera_name = sensor.source_name;
  stream.frame_id = sensor.frame_id;
  stream.topic = sensor.topic;
  stream.width = sensor.width;
  stream.height = sensor.height;
  stream.rate_hz = sensor.rate_hz;
  return stream;
}

CameraStreamConfig BuildDepthStream(const SensorConfig& sensor) {
  CameraStreamConfig stream;
  stream.name = sensor.name + "_depth";
  stream.kind = CameraStreamKind::kDepth;
  stream.camera_name = sensor.source_name;
  stream.sensor_name =
      sensor.depth.sensor_name.empty() ? sensor.source_name + "_depth"
                                       : sensor.depth.sensor_name;
  stream.channel_name = stream.sensor_name;
  stream.frame_id =
      sensor.depth.frame_id.empty() ? sensor.frame_id : sensor.depth.frame_id;
  stream.topic =
      sensor.depth.topic.empty() ? DeriveDepthTopic(sensor.topic) : sensor.depth.topic;
  stream.width = sensor.width;
  stream.height = sensor.height;
  stream.rate_hz = sensor.depth.rate_hz > 0.0 ? sensor.depth.rate_hz : sensor.rate_hz;
  stream.compute_rate_hz =
      sensor.depth.compute_rate_hz > 0.0 ? sensor.depth.compute_rate_hz : stream.rate_hz;
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
    AssignIfPresent(sensor_node, "source_name", &sensor.source_name);
    AssignIfPresent(sensor_node, "width", &sensor.width);
    AssignIfPresent(sensor_node, "height", &sensor.height);
    AssignIfPresent(sensor_node, "rate_hz", &sensor.rate_hz);
    const YAML::Node depth_node = sensor_node["depth"];
    AssignIfPresent(depth_node, "enabled", &sensor.depth.enabled);
    AssignIfPresent(depth_node, "frame_id", &sensor.depth.frame_id);
    AssignIfPresent(depth_node, "topic", &sensor.depth.topic);
    AssignIfPresent(depth_node, "sensor_name", &sensor.depth.sensor_name);
    AssignIfPresent(depth_node, "rate_hz", &sensor.depth.rate_hz);
    AssignIfPresent(depth_node, "compute_rate_hz", &sensor.depth.compute_rate_hz);
    AssignIfPresent(depth_node, "worker_threads", &sensor.depth.worker_threads);
    sensors->push_back(std::move(sensor));
  }
}

void ApplyConfigRoot(
    const YAML::Node& root,
    const fs::path& config_path,
    QuadrotorConfig* config,
    bool apply_global_simulation_config = true) {
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
        NormalizeAircraftForwardAxis(
            LoadVector3(model_node["aircraft_forward_axis"], config->model.aircraft_forward_axis));
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
        config->dynamic_obstacle.config_path =
            ResolvePath(config_path, dynamic_obstacle_node["config_path"].as<std::string>()).string();
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
  AssignIfPresent(interfaces_node, "odom_topic", &config->interfaces.odom_topic);
  AssignIfPresent(interfaces_node, "imu_topic", &config->interfaces.imu_topic);
  AssignIfPresent(interfaces_node, "clock_topic", &config->interfaces.clock_topic);

  const YAML::Node frames_node = root["frames"];
  AssignIfPresent(frames_node, "odom", &config->frames.odom);
  AssignIfPresent(frames_node, "base", &config->frames.base);
  AssignIfPresent(frames_node, "imu", &config->frames.imu);

  if (root["sensors"]) {
    LoadSensors(root["sensors"], &config->sensors);
  }
}

std::optional<fs::path> ResolveRobotConfigPath(
    const YAML::Node& root,
    const fs::path& config_path,
    const fs::path& explicit_robot_path,
    bool require_robot_config) {
  if (!explicit_robot_path.empty()) {
    return fs::absolute(explicit_robot_path);
  }

  const YAML::Node robot_config_node = root["robot_config"];
  if (robot_config_node && robot_config_node.IsScalar()) {
    return ResolvePath(config_path, robot_config_node.as<std::string>());
  }

  if (require_robot_config) {
    throw std::runtime_error(
        "Simulation config must define 'robot_config' to select the robot model.");
  }
  return std::nullopt;
}

QuadrotorConfig LoadConfigFile(
    const fs::path& path,
    const fs::path& explicit_robot_path = {},
    bool require_robot_config = false) {
  const fs::path config_path = fs::absolute(path);
  if (!fs::exists(config_path)) {
    throw std::runtime_error("Config file does not exist: " + config_path.string());
  }

  const YAML::Node root = YAML::LoadFile(config_path.string());
  QuadrotorConfig config;
  config.model.scene_xml = DefaultSceneXmlPath();
  ApplyConfigRoot(root, config_path, &config, true);

  const std::optional<fs::path> robot_config_path =
      ResolveRobotConfigPath(root, config_path, explicit_robot_path, require_robot_config);
  if (robot_config_path.has_value()) {
    const fs::path resolved_robot_config_path = fs::absolute(*robot_config_path);
    if (!fs::exists(resolved_robot_config_path)) {
      throw std::runtime_error(
          "Robot config file does not exist: " + resolved_robot_config_path.string());
    }
    ApplyConfigRoot(
        YAML::LoadFile(resolved_robot_config_path.string()),
        resolved_robot_config_path,
        &config,
        false);
  }

  return config;
}

}  // namespace

QuadrotorConfig LoadConfigFromYaml(const std::string& path) {
  return LoadConfigFile(path);
}

QuadrotorConfig LoadConfigFromYaml(
    const std::string& sim_config_path,
    const std::string& robot_config_path) {
  return LoadConfigFile(sim_config_path, robot_config_path, true);
}

std::vector<CameraStreamConfig> BuildCameraStreamConfigs(
    const std::vector<SensorConfig>& sensors) {
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

}  // namespace quadrotor
