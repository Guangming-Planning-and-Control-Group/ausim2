#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "control/motor_mixer.hpp"
#include "controller/se3_controller.hpp"

namespace mujoco {
class Simulate;
}

namespace quadrotor {

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
  std::filesystem::path model_path = "../assets/crazyfile/scene.xml";
  double duration = 0.0;
  double dt = 0.001;
  double print_interval = 0.5;
  int control_mode = 2;
  int example_mode = 1;
};

struct ViewerConfig {
  bool enabled = true;
  bool fallback_to_headless = true;
  bool mjui_enabled = true;
  bool vsync = true;
};

struct QuadrotorConfig {
  VehicleParams vehicle;
  ControllerGains controller;
  HoverGoal hover_goal;
  CircleTrajectoryConfig circle_trajectory;
  SimulationConfig simulation;
  ViewerConfig viewer;
  double torque_scale = 0.001;
};

QuadrotorConfig LoadConfigFromYaml(const std::string& path);

class QuadrotorSim {
 public:
  explicit QuadrotorSim(QuadrotorConfig config);
  ~QuadrotorSim();

  void LoadModel();
  void Step();
  void Run();
  bool viewer_enabled() const { return viewer_ != nullptr; }

  const mjModel* model() const { return model_; }
  const mjData* data() const { return data_; }

 private:
  static void ControlCallback(const mjModel* model, mjData* data);
  void ApplyControl(const mjModel* model, mjData* data);

  State ReadCurrentState(const mjData* data) const;
  State BuildGoalState(double time, const State& current, Eigen::Vector3d* forward) const;
  void ResolveActuatorIds();
  double CalcMotorForce(double krpm) const;
  double CalcMotorInput(double krpm) const;
  void ResetSimulation();
  void LogStateIfNeeded(
      const mjData* data,
      const State& current,
      const State& goal,
      const Eigen::Vector4d& motor_speed) const;

  void RunHeadless();
  void RunWithViewer();
  void InitializeViewer();
  void CleanupViewer();
  void PhysicsThreadMain();
  void PhysicsLoop(mujoco::Simulate& sim);
  bool LoadModelIntoViewer(
      mujoco::Simulate& sim,
      const std::filesystem::path& model_path,
      bool replace_existing);
  void InstallModelPointers(
      mjModel* new_model,
      mjData* new_data,
      const std::filesystem::path& model_path,
      bool replace_existing);
  void ConfigureDefaultCamera();
  std::string ValidateModel(const mjModel* candidate) const;
  int ComputeControlDecimation() const;
  bool ShouldContinueHeadless() const;
  void SleepToMatchRealtime(
      const std::chrono::high_resolution_clock::time_point& step_start,
      double simulated_seconds) const;
  static void HandleSigint(int signal);

  QuadrotorConfig config_;
  SE3Controller controller_;
  MotorMixer mixer_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::unique_ptr<mujoco::Simulate> viewer_;
  std::thread physics_thread_;
  mjvCamera camera_;
  mjvOption visualization_options_;
  mjvPerturb perturbation_;
  std::array<int, 4> actuator_ids_ = {-1, -1, -1, -1};
  mutable double next_log_time_ = 0.0;
  int vehicle_body_id_ = -1;
  int control_decimation_ = 1;
  int control_step_count_ = 0;
  ControlCommand cached_command_;
  std::atomic_bool stop_requested_ = false;

  static QuadrotorSim* active_instance_;
};

}  // namespace quadrotor
