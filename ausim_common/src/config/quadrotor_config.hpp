#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ausim {

struct VehicleIdentity {
  std::string vehicle_id = "quadrotor";
  std::string ros_namespace = "/quadrotor";
  std::string frame_prefix = "quadrotor";
};

struct RobotConfig {
  int count = 1;
};

struct VehicleParams {
  double mass = 0.033;
  double Ct = 3.25e-4;
  double Cd = 7.9379e-6;
  double arm_length = 0.065 / 2.0;
  double max_thrust = 0.1573;
  double max_torque = 3.842e-03;
  double max_speed_krpm = 22.0;
};

struct ControllerGains {
  double kx = 0.6;
  double kv = 0.4;
  double kR = 6.0;
  double kw = 1.0;
  double rate_hz = 250.0;
};

struct HoverGoal {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.3);
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d heading = Eigen::Vector3d(1.0, 0.0, 0.0);
};

struct CircleTrajectoryConfig {
  double wait_time = 1.5;
  double height = 0.3;
  double radius = 0.5;
  double speed_hz = 0.3;
  double height_gain = 1.5;
};

struct SimulationConfig {
  double duration = 0.0;
  double dt = 0.001;
  double print_interval = 0.5;
  int control_mode = 2;
  int example_mode = 1;
  std::string track_camera_name = "track";
};

struct DynamicObstacleConfig {
  bool enabled = false;
  std::string config_path = "";  // Path to obstacle.yaml
};

struct ViewerConfig {
  bool enabled = true;
  bool fallback_to_headless = true;
  bool mjui_enabled = true;
  bool vsync = true;
};

struct ModelConfig {
  std::filesystem::path scene_xml = "../assets/crazyfile/scene.xml";
  std::string vehicle_body_name = "cf2";
  Eigen::Vector3d aircraft_forward_axis = Eigen::Vector3d(0.0, 1.0, 0.0);
};

struct ActuatorBindingConfig {
  std::array<std::string, 4> motor_names = {"motor1", "motor2", "motor3", "motor4"};
};

struct StateBindingConfig {
  std::string gyro_sensor_name = "body_gyro";
  std::string accelerometer_sensor_name = "body_linacc";
  std::string quaternion_sensor_name = "body_quat";
};

struct Ros2Config {
  std::string node_name = "sim_bridge";
  bool use_sim_time = true;
  double publish_rate_hz = 100.0;
  double command_timeout = 0.5;
  bool publish_tf = true;
  bool publish_clock = true;
};

struct RosInterfaceConfig {
  std::string cmd_vel_topic = "cmd_vel";
  std::string odom_topic = "odom";
  std::string imu_topic = "imu/data";
  std::string clock_topic = "/clock";
  std::string reset_service = "";
  std::string takeoff_service = "";
  std::string teleop_event_topic = "";
  std::string robot_mode_topic = "";
};

struct RosFrameConfig {
  std::string odom = "odom";
  std::string base = "base_link";
  std::string imu = "base_link";
};

struct CameraDepthConfig {
  bool enabled = false;
  std::string frame_id;
  std::string topic;
  std::string sensor_name;
  std::string data_type;
  double rate_hz = 0.0;
  double compute_rate_hz = 0.0;
  int worker_threads = 0;
};

struct SensorConfig {
  std::string name;
  std::string type;
  bool enabled = false;
  std::string frame_id;
  std::string topic;
  std::string source_name;
  int width = 320;
  int height = 240;
  double rate_hz = 30.0;
  CameraDepthConfig depth;
};

enum class CameraStreamKind {
  kColor,
  kDepth,
};

struct CameraStreamConfig {
  std::string name;
  CameraStreamKind kind = CameraStreamKind::kColor;
  std::string channel_name;
  std::string camera_name;
  std::string sensor_name;
  std::string data_type = "distance_to_image_plane_inf_zero";
  std::string frame_id;
  std::string topic;
  int width = 320;
  int height = 240;
  double rate_hz = 30.0;
  double compute_rate_hz = 0.0;
  int worker_threads = 0;
};

struct RobotModeStateConfig {
  std::string name;
  std::string top_state = "SAFE";
  bool accepts_motion = false;
};

struct RobotModeTransitionConfig {
  std::string from;
  std::string to;
  std::string event;
  std::string condition;
  std::string guard;
  std::string action;
};

struct RobotModeConfig {
  std::string initial_state;
  std::vector<RobotModeStateConfig> states;
  std::vector<RobotModeTransitionConfig> transitions;
};

struct QuadrotorConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotConfig robot;
  VehicleIdentity identity;
  ModelConfig model;
  VehicleParams vehicle;
  ControllerGains controller;
  HoverGoal hover_goal;
  CircleTrajectoryConfig circle_trajectory;
  SimulationConfig simulation;
  ViewerConfig viewer;
  DynamicObstacleConfig dynamic_obstacle;
  ActuatorBindingConfig actuators;
  StateBindingConfig state;
  Ros2Config ros2;
  RosInterfaceConfig interfaces;
  RosFrameConfig frames;
  std::vector<SensorConfig> sensors;
  RobotModeConfig teleop_mode;
  double torque_scale = 0.001;
};

using SimConfig = QuadrotorConfig;

QuadrotorConfig LoadConfigFromYaml(const std::string& path);
QuadrotorConfig LoadConfigFromYaml(const std::string& sim_config_path, const std::string& robot_config_path);
std::vector<CameraStreamConfig> BuildCameraStreamConfigs(const std::vector<SensorConfig>& sensors);

}  // namespace ausim

namespace quadrotor {

using ::ausim::ActuatorBindingConfig;
using ::ausim::BuildCameraStreamConfigs;
using ::ausim::CameraDepthConfig;
using ::ausim::CameraStreamConfig;
using ::ausim::CameraStreamKind;
using ::ausim::CircleTrajectoryConfig;
using ::ausim::ControllerGains;
using ::ausim::DynamicObstacleConfig;
using ::ausim::HoverGoal;
using ::ausim::LoadConfigFromYaml;
using ::ausim::ModelConfig;
using ::ausim::QuadrotorConfig;
using ::ausim::RobotConfig;
using ::ausim::Ros2Config;
using ::ausim::RosFrameConfig;
using ::ausim::RosInterfaceConfig;
using ::ausim::RobotModeConfig;
using ::ausim::RobotModeStateConfig;
using ::ausim::RobotModeTransitionConfig;
using ::ausim::SensorConfig;
using ::ausim::SimConfig;
using ::ausim::SimulationConfig;
using ::ausim::StateBindingConfig;
using ::ausim::VehicleIdentity;
using ::ausim::VehicleParams;
using ::ausim::ViewerConfig;

}  // namespace quadrotor
