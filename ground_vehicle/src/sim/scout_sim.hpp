#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <string>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "config/scout_config.hpp"
#include "control/differential_drive_controller.hpp"
#include "runtime/runtime_types.hpp"
#include "sim/wheel_actuator_writer.hpp"

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
  void PublishTelemetry(bool log_state = true);
  void LogStateIfNeeded(const quadrotor::TelemetrySnapshot& snapshot) const;
  bool ShouldContinue() const;
  void SleepToMatchRealtime(
      const std::chrono::high_resolution_clock::time_point& step_start) const;
  static void HandleSigint(int signal);

  ScoutConfig config_;
  DifferentialDriveController controller_;
  WheelActuatorWriter actuator_writer_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  int freejoint_qpos_adr_ = -1;
  int freejoint_dof_adr_ = -1;
  SensorBinding gyro_sensor_;
  SensorBinding accelerometer_sensor_;
  SensorBinding quaternion_sensor_;
  WheelSpeeds last_wheel_speeds_;
  std::string last_command_source_ = "hold";
  bool last_command_valid_ = false;
  mutable double next_log_time_ = 0.0;
  std::atomic_bool stop_requested_ = false;

  static ScoutSim* active_instance_;
};

}  // namespace ground_vehicle
