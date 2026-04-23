#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <Eigen/Core>

#include "config/quadrotor_config.hpp"
#include "runtime/vehicle_runtime.hpp"
#include "sim/dynamic_obstacle_runtime.hpp"
#include "sim/camera_renderer.hpp"
#include "sim/mujoco_actuator_writer.hpp"
#include "sim/mujoco_bindings.hpp"
#include "sim/mujoco_state_reader.hpp"

namespace mujoco {
class Simulate;
}

namespace quadrotor {

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
  struct CameraStreamRuntime;

  static void ControlCallback(const mjModel* model, mjData* data);
  void ApplyControl(const mjModel* model, mjData* data);
  bool HandleDiscreteCommand(const DiscreteCommand& command, const RuntimeInput& input);
  void ProcessPendingResetSimulation();
  void UpdateViewerModeOverlay();

  void ResetSimulation();
  void LogStateIfNeeded(const TelemetrySnapshot& snapshot) const;

  void RunHeadless();
  void RunWithViewer();
  void InitializeViewer();
  void CleanupViewer();
  void PhysicsThreadMain();
  void PhysicsLoop(mujoco::Simulate& sim);
  bool LoadModelIntoViewer(mujoco::Simulate& sim, const std::filesystem::path& model_path, bool replace_existing);
  void InstallModelPointers(mjModel* new_model, mjData* new_data, const std::filesystem::path& model_path, bool replace_existing);
  void ConfigureDefaultCamera();
  void ResolveCameraSensors(const mjModel* model, const mjData* data = nullptr);
  void ApplyDepthPluginPerformanceConfig(mjModel* model) const;
  void InitializeCameraRendering();
  void RefreshCameraRendering();
  void RenderCameraFramesIfNeeded();
  void InitializeVisualizationState();
  std::string ValidateModel(const mjModel* candidate) const;
  int ComputeControlDecimation() const;
  bool ShouldContinueHeadless() const;
  void SleepToMatchRealtime(const std::chrono::high_resolution_clock::time_point& step_start, double simulated_seconds) const;
  static void HandleSigint(int signal);
  bool IsDepthStreamRenderable(const CameraStreamRuntime& stream) const;
  bool HasRenderableDepthStream() const;
  bool HasDueDepthStreamAfterStep(double next_sim_time) const;

  // Dynamic obstacle management
  void InitializeDynamicObstacleManager();
  bool PrepareDynamicObstaclesForStep();
  void PublishDynamicObstaclesSnapshotIfDue();

  QuadrotorConfig config_;
  VehicleRuntime runtime_;
  MujocoBindings bindings_;
  MujocoStateReader state_reader_;
  MujocoActuatorWriter actuator_writer_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  std::unique_ptr<mujoco::Simulate> viewer_;
  std::thread physics_thread_;
  mjvCamera camera_{};
  mjvOption visualization_options_{};
  mjvPerturb perturbation_{};
  mutable double next_log_time_ = 0.0;
  int control_decimation_ = 1;
  int control_step_count_ = 0;
  std::atomic_bool stop_requested_ = false;
  bool visualization_state_initialized_ = false;
  struct CameraStreamRuntime {
    std::string name;
    CameraStreamKind kind = CameraStreamKind::kColor;
    std::string channel_name;
    std::string camera_name;
    std::string sensor_name;
    std::string data_type = "distance_to_image_plane_inf_zero";
    int width = 320;
    int height = 240;
    double period_seconds = 1.0 / 30.0;
    double compute_period_seconds = 1.0 / 30.0;
    int worker_threads = 0;
    double next_render_time = 0.0;
    int camera_id = -1;
    int sensor_id = -1;
    int sensor_data_adr = -1;
    int sensor_data_size = 0;
    std::uint32_t sequence = 0;
  };
  std::vector<CameraStreamRuntime> camera_streams_;
  CameraRenderer camera_renderer_;
  bool camera_rendering_ready_ = false;
  bool camera_rendering_failed_ = false;
  bool pending_reset_simulation_ = false;
  std::uint64_t last_discrete_command_sequence_ = 0;
  DiscreteCommandAckStatus last_discrete_command_status_ = DiscreteCommandAckStatus::kNone;

  ausim::DynamicObstacleRuntime dynamic_obstacle_runtime_;
  double next_dynamic_obstacle_publish_time_ = 0.0;

  static QuadrotorSim* active_instance_;
};

}  // namespace quadrotor
