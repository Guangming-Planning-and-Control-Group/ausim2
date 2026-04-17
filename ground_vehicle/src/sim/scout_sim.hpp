#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <Eigen/Core>

#include "config/scout_config.hpp"
#include "control/differential_drive_controller.hpp"
#include "runtime/robot_mode_state_machine.hpp"
#include "runtime/runtime_types.hpp"
#include "sim/wheel_actuator_writer.hpp"

namespace mujoco {
class Simulate;
}

namespace ground_vehicle {

struct SensorBinding {
  std::string name;
  int id = -1;
  int adr = -1;
  int dim = 0;
  bool resolved = false;
};

class ScoutSim {
 public:
  explicit ScoutSim(ScoutConfig config);
  ~ScoutSim();

  void LoadModel();
  void Step();
  void Run();

 private:
  void ResolveBindings();
  void ResolveSensor(SensorBinding* binding) const;
  void ApplyControl();
  bool HandleDiscreteCommand(const ausim::DiscreteCommand& command);
  void PublishTelemetry(bool log_state = true);
  void LogStateIfNeeded(const ausim::TelemetrySnapshot& snapshot) const;
  void RunHeadless();
  void RunWithViewer();
  void InitializeVisualizationState();
  void ConfigureDefaultCamera();
  void InitializeViewer();
  void CleanupViewer();
  void PhysicsThreadMain();
  void PhysicsLoop(mujoco::Simulate& sim);
  bool LoadModelIntoViewer(mujoco::Simulate& sim, const std::filesystem::path& model_path, bool replace_existing);
  void InstallModelPointers(mjModel* new_model, mjData* new_data, const std::filesystem::path& model_path, bool replace_existing);
  void ResetSimulation();
  bool ShouldContinue() const;
  void SleepToMatchRealtime(const std::chrono::high_resolution_clock::time_point& step_start) const;
  static void HandleSigint(int signal);
  static bool MotionCommandActive(const ausim::VelocityCommand& command);

  ScoutConfig config_;
  DifferentialDriveController controller_;
  WheelActuatorWriter actuator_writer_;
  ausim::RobotModeStateMachine mode_machine_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::unique_ptr<mujoco::Simulate> viewer_;
  std::thread physics_thread_;
  mjvCamera camera_{};
  mjvOption visualization_options_{};
  mjvPerturb perturbation_{};
  int freejoint_qpos_adr_ = -1;
  int freejoint_dof_adr_ = -1;
  SensorBinding gyro_sensor_;
  SensorBinding accelerometer_sensor_;
  SensorBinding quaternion_sensor_;
  WheelSpeeds last_wheel_speeds_;
  std::string last_command_source_ = "hold";
  bool last_command_valid_ = false;
  std::uint64_t last_discrete_command_sequence_ = 0;
  ausim::DiscreteCommandAckStatus last_discrete_command_status_ = ausim::DiscreteCommandAckStatus::kNone;
  mutable double next_log_time_ = 0.0;
  std::atomic_bool stop_requested_ = false;
  bool visualization_state_initialized_ = false;

  static ScoutSim* active_instance_;
};

}  // namespace ground_vehicle
