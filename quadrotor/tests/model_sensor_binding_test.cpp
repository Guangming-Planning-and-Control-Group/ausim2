#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "config/quadrotor_config.hpp"
#include "sim/quadrotor_sim.hpp"

namespace fs = std::filesystem;

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

fs::path ResolveRepoRoot() {
  fs::path current = fs::current_path();
  for (int i = 0; i < 4; ++i) {
    if (fs::exists(current / "quadrotor" / "cfg" / "sim_config.yaml")) {
      return current;
    }
    if (!current.has_parent_path()) {
      break;
    }
    current = current.parent_path();
  }
  return {};
}

}  // namespace

int main() {
  const fs::path repo_root = ResolveRepoRoot();
  Expect(!repo_root.empty(), "failed to locate repo root for model_sensor_binding_test");

  const fs::path robot_config_path = fs::temp_directory_path() / "model_sensor_binding_robot.yaml";
  std::ofstream output(robot_config_path);
  output << R"yaml(
robot:
  count: 1

identity:
  vehicle_id: cf2
  namespace: /uav1
  frame_prefix: uav1

model:
  scene_xml: ../../../assets/crazyfile/scene.xml
  body_name: cf2
  aircraft_forward_axis: [1.0, 0.0, 0.0]

bindings:
  actuators:
    motor_names: [motor1, motor2, motor3, motor4]
  state:
    gyro_sensor: body_gyro
    accel_sensor: body_linacc
    quat_sensor: body_quat

vehicle:
  mass: 0.033
  thrust_coefficient: 3.25e-4
  drag_coefficient: 7.9379e-6
  arm_length: 0.0325
  max_thrust: 0.1573
  max_torque: 0.003842
  max_speed_krpm: 22.0

controller:
  rate_hz: 250.0
  kx: 0.6
  kv: 0.4
  kR: 6.0
  kw: 1.0
  torque_scale: 0.001

interfaces:
  cmd_vel_topic: cmd_vel
  joy_cmd_vel_topic: /joy/cmd_vel
  odom_topic: odom
  imu_topic: imu/data
  clock_topic: /clock
  robot_mode_topic: teleop/mode

frames:
  odom: odom
  base: base_link
  imu: base_link

sensors:
  - name: front_camera
    type: camera
    enabled: true
    frame_id: camera_link
    topic: camera/image_raw
    rate_hz: 30.0
    depth:
      enabled: true
)yaml";
  output.close();

  quadrotor::QuadrotorConfig config =
      quadrotor::LoadConfigFromYaml((repo_root / "quadrotor" / "cfg" / "sim_config.yaml").string(), robot_config_path.string());
  config.viewer.enabled = false;
  config.viewer.fallback_to_headless = true;
  config.model.scene_xml = repo_root / "assets" / "crazyfile" / "scene.xml";

  quadrotor::QuadrotorSim sim(config);
  sim.LoadModel();

  Expect(sim.model() != nullptr, "expected model to load with MJCF-derived camera bindings");
  Expect(sim.data() != nullptr, "expected data to be allocated");

  fs::remove(robot_config_path);
  return 0;
}
