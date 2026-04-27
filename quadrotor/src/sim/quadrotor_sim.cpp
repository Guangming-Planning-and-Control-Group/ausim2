#include "sim/quadrotor_sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>

#include "mujoco-3.6.0/simulate/glfw_adapter.h"
#include "mujoco-3.6.0/simulate/simulate.h"
#include "runtime/data_board_interface.hpp"

namespace fs = std::filesystem;

namespace quadrotor {

QuadrotorSim* QuadrotorSim::active_instance_ = nullptr;

namespace {
namespace mj = ::mujoco;

using Seconds = std::chrono::duration<double>;

constexpr double kSyncMisalign = 0.1;

#ifndef QUADROTOR_MUJOCO_PLUGIN_FALLBACK_DIR
  #define QUADROTOR_MUJOCO_PLUGIN_FALLBACK_DIR ""
#endif

std::optional<fs::path> ResolveBundledPluginDirectory() {
#if defined(__linux__)
  std::error_code error;
  const fs::path executable_path = fs::read_symlink("/proc/self/exe", error);
  if (!error && !executable_path.empty()) {
    const fs::path plugin_dir = executable_path.parent_path() / "mujoco_plugin";
    if (fs::exists(plugin_dir, error) && fs::is_directory(plugin_dir, error)) {
      return plugin_dir;
    }
  }
#endif
  return std::nullopt;
}

bool ContainsDecoderPlugin(const fs::path& directory) {
  std::error_code error;
  return fs::exists(directory / "libobj_decoder.so", error) || fs::exists(directory / "libstl_decoder.so", error);
}

void AppendPluginDirectory(std::vector<fs::path>& directories, const fs::path& directory, bool skip_decoder_directory = false) {
  if (directory.empty()) {
    return;
  }

  std::error_code error;
  if (!fs::exists(directory, error) || !fs::is_directory(directory, error)) {
    return;
  }
  if (skip_decoder_directory && ContainsDecoderPlugin(directory)) {
    return;
  }

  for (const fs::path& existing : directories) {
    if (existing == directory) {
      return;
    }
  }
  directories.push_back(directory);
}

void AppendPluginDirectoriesFromEnv(std::vector<fs::path>& directories, bool skip_decoder_directories) {
  const char* plugin_dirs = std::getenv("MUJOCO_PLUGIN_DIR");
  if (plugin_dirs == nullptr || plugin_dirs[0] == '\0') {
    return;
  }

  std::string token;
  std::stringstream stream(plugin_dirs);
  while (std::getline(stream, token, ':')) {
    if (!token.empty()) {
      AppendPluginDirectory(directories, fs::path(token), skip_decoder_directories);
    }
  }
}

void LoadMuJoCoPlugins() {
  std::vector<fs::path> plugin_directories;
  const fs::path fallback_decoder_dir(QUADROTOR_MUJOCO_PLUGIN_FALLBACK_DIR);
  const bool fallback_has_decoders = !fallback_decoder_dir.empty() && ContainsDecoderPlugin(fallback_decoder_dir);

  AppendPluginDirectory(plugin_directories, fallback_decoder_dir);
  AppendPluginDirectoriesFromEnv(plugin_directories, fallback_has_decoders);

  if (const auto bundled_plugin_dir = ResolveBundledPluginDirectory()) {
    AppendPluginDirectory(plugin_directories, *bundled_plugin_dir);
  }

  for (const fs::path& plugin_directory : plugin_directories) {
    const std::string plugin_dir_string = plugin_directory.string();
    mj_loadAllPluginLibraries(plugin_dir_string.c_str(), nullptr);
  }
}

constexpr double kSimRefreshFraction = 0.7;
constexpr int kLoadErrorLength = 1024;
constexpr double kFrequencyTolerance = 1e-6;
constexpr std::string_view kDefaultRayCasterDepthDataType = "distance_to_image_plane_inf_zero";

struct RayCasterDepthLayout {
  int width = 0;
  int height = 0;
  int data_point = 0;
  int data_size = 0;
};

struct MutablePluginConfigSlot {
  char* value = nullptr;
  std::size_t capacity = 0;
};

template <typename StreamType>
bool HasColorCameraStreams(const std::vector<StreamType>& streams) {
  for (const auto& stream : streams) {
    if (stream.kind == CameraStreamKind::kColor) {
      return true;
    }
  }
  return false;
}

std::string CameraStreamBaseName(const std::string& stream_name) {
  constexpr std::string_view kDepthSuffix = "_depth";
  if (stream_name.size() >= kDepthSuffix.size() &&
      stream_name.compare(stream_name.size() - kDepthSuffix.size(), kDepthSuffix.size(), kDepthSuffix) == 0) {
    return stream_name.substr(0, stream_name.size() - kDepthSuffix.size());
  }
  return stream_name;
}

std::vector<int> CollectPublishableCameraIds(const mjModel* model, const std::string& track_camera_name) {
  std::vector<int> camera_ids;
  if (model == nullptr) {
    return camera_ids;
  }

  const int track_camera_id = track_camera_name.empty() ? -1 : mj_name2id(model, mjOBJ_CAMERA, track_camera_name.c_str());
  for (int camera_id = 0; camera_id < model->ncam; ++camera_id) {
    if (camera_id != track_camera_id) {
      camera_ids.push_back(camera_id);
    }
  }
  return camera_ids;
}

std::vector<int> CollectPluginCameraSensorIds(const mjModel* model, int camera_id) {
  std::vector<int> sensor_ids;
  if (model == nullptr || camera_id < 0) {
    return sensor_ids;
  }

  for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
    if (model->sensor_type[sensor_id] == mjSENS_PLUGIN && model->sensor_objtype[sensor_id] == mjOBJ_CAMERA &&
        model->sensor_objid[sensor_id] == camera_id) {
      sensor_ids.push_back(sensor_id);
    }
  }
  return sensor_ids;
}

std::optional<RayCasterDepthLayout> ReadRayCasterDepthLayout(const mjModel* model, const mjData* data, int sensor_id) {
  if (model == nullptr || data == nullptr || sensor_id < 0 || sensor_id >= model->nsensor) {
    return std::nullopt;
  }

  if (model->sensor_type[sensor_id] != mjSENS_PLUGIN) {
    return std::nullopt;
  }

  RayCasterDepthLayout layout;
  layout.data_size = model->sensor_dim[sensor_id];
  const int plugin_instance = model->sensor_plugin[sensor_id];
  if (plugin_instance < 0 || plugin_instance >= model->nplugin) {
    return layout;
  }

  const int state_idx = model->plugin_stateadr[plugin_instance];
  const int state_num = model->plugin_statenum[plugin_instance];
  if (state_num >= 2) {
    layout.width = static_cast<int>(std::lround(data->plugin_state[state_idx]));
    layout.height = static_cast<int>(std::lround(data->plugin_state[state_idx + 1]));
  }
  if (state_num >= 4) {
    layout.data_point = static_cast<int>(std::lround(data->plugin_state[state_idx + 2]));
    layout.data_size = static_cast<int>(std::lround(data->plugin_state[state_idx + 3]));
  }
  return layout;
}

std::optional<MutablePluginConfigSlot> FindMutablePluginConfigSlot(mjModel* model, int plugin_instance, const char* attribute_name) {
  if (model == nullptr || plugin_instance < 0 || plugin_instance >= model->nplugin || attribute_name == nullptr) {
    return std::nullopt;
  }

  const mjpPlugin* plugin = mjp_getPluginAtSlot(model->plugin[plugin_instance]);
  if (plugin == nullptr) {
    return std::nullopt;
  }

  char* ptr = model->plugin_attr + model->plugin_attradr[plugin_instance];
  for (int i = 0; i < plugin->nattribute; ++i) {
    const std::size_t capacity = std::strlen(ptr) + 1;
    if (std::strcmp(plugin->attributes[i], attribute_name) == 0) {
      return MutablePluginConfigSlot{ptr, capacity};
    }
    ptr += capacity;
  }
  return std::nullopt;
}

bool IsSingleRayCasterDataType(std::string_view data_type) { return data_type.find_first_of(" \t\n\r\f\v") == std::string_view::npos; }

bool IsVectorRayCasterDataType(std::string_view data_type) {
  return data_type.find("pos_w") != std::string_view::npos || data_type.find("pos_b") != std::string_view::npos;
}

bool IsScalarRayCasterDataType(std::string_view data_type) {
  if (data_type.empty() || IsVectorRayCasterDataType(data_type)) {
    return false;
  }
  return data_type.find("distance_to_image_plane") != std::string_view::npos || data_type.find("image_plane_image") != std::string_view::npos ||
         data_type.find("image_plane_normal") != std::string_view::npos || data_type.find("data") != std::string_view::npos ||
         data_type.find("image") != std::string_view::npos || data_type.find("normal") != std::string_view::npos;
}

std::string ResolveRayCasterDepthDataType(const std::string& configured_data_type) {
  const std::string data_type = configured_data_type.empty() ? std::string(kDefaultRayCasterDepthDataType) : configured_data_type;
  if (!IsSingleRayCasterDataType(data_type)) {
    throw std::runtime_error(
        "Ray-caster depth data_type must be a single sensor_data_types token. "
        "The current ROS depth image path publishes one 32FC1 block.");
  }
  if (!IsScalarRayCasterDataType(data_type)) {
    throw std::runtime_error("Ray-caster depth data_type '" + data_type +
                             "' is not supported by the current 32FC1 depth image path. "
                             "Use a scalar type such as data, image, normal, distance_to_image_plane, "
                             "image_plane_image, or image_plane_normal; pos_w/pos_b require a separate xyz stream.");
  }
  return data_type;
}

void OverwritePluginTextConfig(mjModel* model, int plugin_instance, const char* attribute_name, const std::string& text,
                               const std::string& stream_name);

void OverwritePluginIntConfig(mjModel* model, int plugin_instance, const char* attribute_name, int value, const std::string& stream_name) {
  OverwritePluginTextConfig(model, plugin_instance, attribute_name, std::to_string(value), stream_name);
}

void OverwritePluginTextConfig(mjModel* model, int plugin_instance, const char* attribute_name, const std::string& text,
                               const std::string& stream_name) {
  const std::optional<MutablePluginConfigSlot> slot = FindMutablePluginConfigSlot(model, plugin_instance, attribute_name);
  if (!slot.has_value()) {
    throw std::runtime_error("Depth stream '" + stream_name + "' could not find plugin attribute '" + attribute_name + "'.");
  }

  if (text.size() + 1 > slot->capacity) {
    throw std::runtime_error("Depth stream '" + stream_name + "' needs a wider placeholder for plugin attribute '" + attribute_name +
                             "' in the MJCF.");
  }

  std::memset(slot->value, ' ', slot->capacity - 1);
  std::memcpy(slot->value, text.data(), text.size());
  slot->value[slot->capacity - 1] = '\0';
}

int ComputeStepUpdate(double physics_dt, double compute_period_seconds) {
  if (physics_dt <= 0.0 || compute_period_seconds <= physics_dt) {
    return 1;
  }
  return std::max(1, static_cast<int>(std::lround(compute_period_seconds / physics_dt)));
}

std::string VectorToString(const Eigen::Vector4d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ", " << vector[3] << ")";
  return stream.str();
}

std::string VectorToString(const Eigen::Vector3d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
  return stream.str();
}

void TrimTrailingNewline(char* buffer) {
  if (buffer == nullptr) {
    return;
  }
  const std::size_t length = std::strlen(buffer);
  if (length > 0 && buffer[length - 1] == '\n') {
    buffer[length - 1] = '\0';
  }
}

void SetLoadError(mj::Simulate& sim, const std::string& message) { std::snprintf(sim.load_error, sizeof(sim.load_error), "%s", message.c_str()); }

bool HasGraphicalDisplay() {
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

bool CanInitializeOfficialViewer(std::string* reason) {
  if (!HasGraphicalDisplay()) {
    if (reason != nullptr) {
      *reason = "no graphical display detected";
    }
    return false;
  }

  if (!glfwInit()) {
    if (reason != nullptr) {
      *reason = "could not initialize GLFW";
    }
    return false;
  }

  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* probe_window = glfwCreateWindow(64, 64, "quadrotor-viewer-probe", nullptr, nullptr);
  if (probe_window == nullptr) {
    glfwTerminate();
    if (reason != nullptr) {
      *reason = "could not create GLFW window";
    }
    return false;
  }

  glfwDestroyWindow(probe_window);
  glfwTerminate();
  return true;
}

struct ModelLoadResult {
  mjModel* model = nullptr;
  std::string message;
  bool pause_after_load = false;
};

ModelLoadResult LoadModelFile(const fs::path& path) {
  LoadMuJoCoPlugins();
  ModelLoadResult result;

  char load_error[kLoadErrorLength] = "Could not load binary model";
  const auto load_start = mj::Simulate::Clock::now();

  if (path.extension() == ".mjb") {
    result.model = mj_loadModel(path.string().c_str(), nullptr);
    if (!result.model) {
      result.message = load_error;
      return result;
    }
    load_error[0] = '\0';
  } else {
    result.model = mj_loadXML(path.string().c_str(), nullptr, load_error, sizeof(load_error));
    TrimTrailingNewline(load_error);
    if (!result.model) {
      result.message = load_error;
      return result;
    }
  }

  const double load_seconds = Seconds(mj::Simulate::Clock::now() - load_start).count();

  if (load_error[0] != '\0') {
    result.pause_after_load = true;
    result.message = load_error;
  } else if (load_seconds > 0.25) {
    char message[kLoadErrorLength];
    std::snprintf(message, sizeof(message), "Model loaded in %.2g seconds", load_seconds);
    result.message = message;
  }

  return result;
}

const char* Diverged(int disableflags, const mjData* data) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning warning : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (data->warning[warning].number > 0) {
        return mju_warningText(warning, data->warning[warning].lastinfo);
      }
    }
  }
  return nullptr;
}

}  // namespace

QuadrotorSim::QuadrotorSim(QuadrotorConfig config)
    : config_(std::move(config)), runtime_(config_), bindings_(config_), actuator_writer_(config_.vehicle) {
  ClearVelocityCommand();
  ClearDiscreteCommand();
  control_decimation_ = ComputeControlDecimation();

  if (config_.robot.count <= 0) {
    throw std::runtime_error("robot.count must be a positive integer.");
  }
  if (config_.robot.count != 1) {
    throw std::runtime_error("robot.count > 1 is not implemented yet. Current runtime supports exactly one simulated vehicle.");
  }

  for (const CameraStreamConfig& stream_config : BuildCameraStreamConfigs(config_.sensors)) {
    if (stream_config.rate_hz <= 0.0) {
      throw std::runtime_error("Camera stream '" + stream_config.name + "' must define a positive rate_hz.");
    }

    CameraStreamRuntime stream;
    stream.name = stream_config.name;
    stream.kind = stream_config.kind;
    stream.channel_name = stream_config.channel_name;
    stream.data_type = stream_config.data_type;
    stream.period_seconds = 1.0 / stream_config.rate_hz;
    stream.compute_period_seconds = stream_config.compute_rate_hz > 0.0 ? 1.0 / stream_config.compute_rate_hz : stream.period_seconds;
    stream.worker_threads = stream_config.worker_threads;
    camera_streams_.push_back(std::move(stream));
  }
}

QuadrotorSim::~QuadrotorSim() {
  if (active_instance_ == this) {
    mjcb_control = nullptr;
    active_instance_ = nullptr;
  }
  CleanupViewer();
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
}

void QuadrotorSim::LoadModel() {
  const ModelLoadResult load_result = LoadModelFile(config_.model.scene_xml);
  if (load_result.model == nullptr) {
    throw std::runtime_error("Failed to load model: " + load_result.message);
  }

  const std::string validation_error = ValidateModel(load_result.model);
  if (!validation_error.empty()) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error(validation_error);
  }

  ResolveCameraSensors(load_result.model, nullptr);
  ApplyDepthPluginPerformanceConfig(load_result.model);
  load_result.model->opt.timestep = config_.simulation.dt;
  mjData* new_data = mj_makeData(load_result.model);
  if (new_data == nullptr) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error("Failed to allocate mjData.");
  }

  InstallModelPointers(load_result.model, new_data, config_.model.scene_xml, true);

  if (!load_result.message.empty()) {
    if (load_result.pause_after_load) {
      std::cout << "Model compiled, but simulation warning:\n  " << load_result.message << '\n';
    } else {
      std::cout << load_result.message << '\n';
    }
  }
}

void QuadrotorSim::Step() {
  if (model_ == nullptr || data_ == nullptr) {
    throw std::runtime_error("Simulation is not loaded.");
  }
  PrepareDynamicObstaclesForStep();
  mj_step(model_, data_);
  ProcessPendingResetSimulation();
  PublishDynamicObstaclesSnapshotIfDue();
}

#ifdef AUSIM_TESTING
void QuadrotorSim::TestViewerStepOnce() {
  if (model_ == nullptr || data_ == nullptr) {
    throw std::runtime_error("Simulation is not loaded.");
  }

  PrepareDynamicObstaclesForStep();
  mj_step(model_, data_);
  ProcessPendingResetSimulation();
  FinalizeStep(true, true);
}

void QuadrotorSim::TestViewerPauseTick(bool pause_update) {
  if (model_ == nullptr || data_ == nullptr) {
    throw std::runtime_error("Simulation is not loaded.");
  }

  mj_forward(model_, data_);
  if (pause_update) {
    mju_copy(data_->qacc_warmstart, data_->qacc, model_->nv);
  }
  UpdateViewerModeOverlay();
  PublishDynamicObstaclesSnapshotIfDue();
}
#endif

void QuadrotorSim::Run() {
  stop_requested_.store(false);
  InitializeVisualizationState();

  if (config_.viewer.enabled) {
    std::string viewer_error;
    if (!CanInitializeOfficialViewer(&viewer_error)) {
      if (!config_.viewer.fallback_to_headless) {
        throw std::runtime_error("Official simulate viewer unavailable: " + viewer_error);
      }
      std::cerr << "viewer unavailable, falling back to headless: " << viewer_error << '\n';
    } else {
      InitializeViewer();
    }
  }

  std::signal(SIGINT, &QuadrotorSim::HandleSigint);

  if (viewer_) {
    RunWithViewer();
  } else {
    if (model_ == nullptr || data_ == nullptr) {
      LoadModel();
    }
    std::cout << "Running MuJoCo simulation with model: " << config_.model.scene_xml << '\n';
    std::cout << "Duration: " << config_.simulation.duration << " s, dt: " << config_.simulation.dt << " s\n";
    RunHeadless();
  }

  std::signal(SIGINT, SIG_DFL);

  if (data_ != nullptr) {
    const RuntimeInput final_input = state_reader_.Read(model_, data_, bindings_);
    const State& final_state = final_input.current_state;
    std::cout << "Simulation finished at t=" << std::fixed << std::setprecision(3) << data_->time
              << " s, final position=" << VectorToString(final_state.position) << '\n';
  }
}

void QuadrotorSim::ControlCallback(const mjModel* model, mjData* data) {
  if (active_instance_ != nullptr) {
    active_instance_->ApplyControl(model, data);
  }
}

void QuadrotorSim::ApplyControl(const mjModel* model, mjData* data) {
  const RuntimeInput input = state_reader_.Read(model, data, bindings_);
  if (const std::optional<DiscreteCommand> command = ReadDiscreteCommand();
      command.has_value() && command->sequence != last_discrete_command_sequence_) {
    if (HandleDiscreteCommand(*command, input)) {
      last_discrete_command_sequence_ = command->sequence;
      last_discrete_command_status_ = DiscreteCommandAckStatus::kSuccess;
    } else {
      last_discrete_command_sequence_ = command->sequence;
      last_discrete_command_status_ = DiscreteCommandAckStatus::kUnsupported;
    }
    ClearDiscreteCommand();
  }
  const bool recompute_control = control_step_count_ == 0;
  const double control_dt = control_decimation_ * model->opt.timestep;
  if (recompute_control) {
    runtime_.Tick(control_dt, input);
  }
  const RuntimeOutput output = runtime_.Step(input, recompute_control, control_dt);

  actuator_writer_.WriteMotorInputs(bindings_, data, output.motor_speed_krpm);

  TelemetrySnapshot snapshot;
  snapshot.sim_time = input.sim_time;
  snapshot.state = input.current_state;
  snapshot.imu = input.imu;
  snapshot.goal_state = output.goal.state;
  snapshot.forward = output.goal.forward;
  snapshot.motor_speed_krpm = output.motor_speed_krpm;
  snapshot.goal_source = output.goal.source;
  snapshot.has_goal = true;
  snapshot.robot_mode = runtime_.ModeSnapshot();
  snapshot.last_discrete_command_sequence = last_discrete_command_sequence_;
  snapshot.last_discrete_command_status = last_discrete_command_status_;
  WriteTelemetrySnapshot(snapshot);

  if (recompute_control) {
    LogStateIfNeeded(snapshot);
  }

  control_step_count_ = (control_step_count_ + 1) % control_decimation_;
}

bool QuadrotorSim::HandleDiscreteCommand(const DiscreteCommand& command, const RuntimeInput& input) {
  switch (command.kind) {
    case DiscreteCommandKind::kResetSimulation:
      pending_reset_simulation_ = true;
      ClearVelocityCommand();
      return true;
    case DiscreteCommandKind::kGenericEvent:
      return runtime_.HandleDiscreteCommand(command, input);
    case DiscreteCommandKind::kNone:
      return false;
  }
  return false;
}

void QuadrotorSim::ResetSimulation() {
  if (model_ == nullptr || data_ == nullptr) {
    return;
  }
  pending_reset_simulation_ = false;
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  next_log_time_ = 0.0;
  control_step_count_ = 0;
  runtime_.Reset();
  for (auto& stream : camera_streams_) {
    stream.next_render_time = 0.0;
    stream.sequence = 0;
  }
  next_dynamic_obstacle_publish_time_ = 0.0;
  dynamic_obstacle_runtime_.ResetToCurrentTime();
}

void QuadrotorSim::ProcessPendingResetSimulation() {
  if (!pending_reset_simulation_) {
    return;
  }

  ResetSimulation();

  if (viewer_ != nullptr) {
    viewer_->load_error[0] = '\0';
    viewer_->scrub_index = 0;
    viewer_->speed_changed = true;
    viewer_->pending_.ui_update_simulation = true;
  }
}

void QuadrotorSim::FinalizeStep(bool render_camera_frames, bool update_viewer_overlay) {
  if (render_camera_frames) {
    RenderCameraFramesIfNeeded();
  }
  if (update_viewer_overlay) {
    UpdateViewerModeOverlay();
  }
  PublishDynamicObstaclesSnapshotIfDue();
}

void QuadrotorSim::UpdateViewerModeOverlay() {
  if (viewer_ == nullptr) {
    return;
  }

  viewer_->user_texts_new_.clear();
  if (config_.viewer.show_mode_state_overlay) {
    const RobotModeSnapshot snapshot = runtime_.ModeSnapshot();
    if (!snapshot.sub_state.empty()) {
      viewer_->user_texts_new_.emplace_back(mjFONT_NORMAL, mjGRID_TOPRIGHT, "State", snapshot.sub_state);
    }
  }
  viewer_->newtextrequest.store(1);
}

void QuadrotorSim::RunHeadless() {
  constexpr int kStepsPerCycle = 5;
  InitializeCameraRendering();

  while (ShouldContinueHeadless()) {
    const auto step_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kStepsPerCycle && ShouldContinueHeadless(); ++i) {
      Step();
      RenderCameraFramesIfNeeded();
    }
    SleepToMatchRealtime(step_start, model_->opt.timestep * kStepsPerCycle);
  }
}

void QuadrotorSim::RunWithViewer() {
  std::cout << "Running MuJoCo simulation with official simulate viewer: " << config_.model.scene_xml << '\n';

  physics_thread_ = std::thread(&QuadrotorSim::PhysicsThreadMain, this);
  viewer_->RenderLoop();

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  viewer_.reset();
}

void QuadrotorSim::InitializeViewer() {
  if (viewer_) {
    return;
  }

  viewer_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &camera_, &visualization_options_, &perturbation_,
                                           /* is_passive = */ false);

  const int mjui_enabled = config_.viewer.mjui_enabled ? 1 : 0;
  viewer_->ui0_enable = mjui_enabled;
  viewer_->ui1_enable = mjui_enabled;
  if (!config_.viewer.mjui_enabled) {
    viewer_->help = 0;
    viewer_->info = 0;
    viewer_->profiler = 0;
    viewer_->sensor = 0;
  }

  viewer_->vsync = config_.viewer.vsync ? 1 : 0;
}

void QuadrotorSim::InitializeVisualizationState() {
  if (visualization_state_initialized_) {
    return;
  }

  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&visualization_options_);
  mjv_defaultPerturb(&perturbation_);
  visualization_state_initialized_ = true;
}

void QuadrotorSim::CleanupViewer() {
  if (viewer_) {
    viewer_->exitrequest.store(true);
  }

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  viewer_.reset();
}

void QuadrotorSim::PhysicsThreadMain() {
  try {
    if (viewer_ == nullptr) {
      return;
    }

    if (model_ != nullptr && data_ != nullptr) {
      const std::string displayed_filename = config_.model.scene_xml.string();
      viewer_->Load(model_, data_, displayed_filename.c_str());
      const std::unique_lock<std::recursive_mutex> lock(viewer_->mtx);
      mj_forward(model_, data_);
      SetLoadError(*viewer_, "");
      UpdateViewerModeOverlay();
    } else if (!LoadModelIntoViewer(*viewer_, config_.model.scene_xml, false)) {
      viewer_->exitrequest.store(true);
      return;
    }

    InitializeCameraRendering();
    PhysicsLoop(*viewer_);
    viewer_->exitrequest.store(true);
  } catch (const std::exception& error) {
    if (viewer_) {
      SetLoadError(*viewer_, error.what());
      viewer_->exitrequest.store(true);
    }
  }
}

void QuadrotorSim::PhysicsLoop(mj::Simulate& sim) {
  std::chrono::time_point<mj::Simulate::Clock> sync_cpu;
  mjtNum sync_sim = 0;

  while (!sim.exitrequest.load() && !stop_requested_.load()) {
    if (sim.droploadrequest.load()) {
      const fs::path dropped_file = sim.dropfilename;
      LoadModelIntoViewer(sim, dropped_file, true);
      sim.droploadrequest.store(false);
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      LoadModelIntoViewer(sim, fs::path(sim.filename), true);
    }

    if (config_.simulation.duration > 0.0 && data_ != nullptr && data_->time >= config_.simulation.duration) {
      sim.exitrequest.store(true);
      break;
    }

    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    if (model_ == nullptr || data_ == nullptr) {
      continue;
    }

    if (sim.run) {
      bool stepped = false;
      const auto start_cpu = mj::Simulate::Clock::now();
      const auto elapsed_cpu = start_cpu - sync_cpu;
      const double elapsed_sim = data_->time - sync_sim;
      const double slowdown = 100.0 / sim.percentRealTime[sim.real_time_index];

      const bool misaligned = std::abs(Seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > kSyncMisalign;

      if (elapsed_sim < 0.0 || elapsed_cpu.count() < 0 || sync_cpu.time_since_epoch().count() == 0 || misaligned || sim.speed_changed) {
        sync_cpu = start_cpu;
        sync_sim = data_->time;
        sim.speed_changed = false;

        sim.InjectNoise(sim.key);
        PrepareDynamicObstaclesForStep();
        mj_step(model_, data_);
        ProcessPendingResetSimulation();

        if (const char* message = Diverged(model_->opt.disableflags, data_)) {
          sim.run = 0;
          SetLoadError(sim, message);
        } else {
          FinalizeStep(true, true);
          stepped = true;
        }
      } else {
        bool measured = false;
        const mjtNum previous_sim = data_->time;
        const double refresh_time = kSimRefreshFraction / std::max(1, sim.refresh_rate);

        while (Seconds((data_->time - sync_sim) * slowdown) < mj::Simulate::Clock::now() - sync_cpu &&
               mj::Simulate::Clock::now() - start_cpu < Seconds(refresh_time)) {
          if (!measured && elapsed_sim > 0.0) {
            sim.measured_slowdown = std::chrono::duration<double>(elapsed_cpu).count() / elapsed_sim;
            measured = true;
          }

          sim.InjectNoise(sim.key);
          PrepareDynamicObstaclesForStep();
          mj_step(model_, data_);
          ProcessPendingResetSimulation();

          if (const char* message = Diverged(model_->opt.disableflags, data_)) {
            sim.run = 0;
            SetLoadError(sim, message);
            break;
          }

          FinalizeStep(true, true);
          stepped = true;
          if (data_->time < previous_sim) {
            break;
          }

          if (config_.simulation.duration > 0.0 && data_->time >= config_.simulation.duration) {
            sim.exitrequest.store(true);
            break;
          }
        }
      }

      if (stepped) {
        sim.AddToHistory();
      }
    } else {
      mj_forward(model_, data_);
      if (sim.pause_update) {
        mju_copy(data_->qacc_warmstart, data_->qacc, model_->nv);
      }
      sim.speed_changed = true;
      UpdateViewerModeOverlay();
      PublishDynamicObstaclesSnapshotIfDue();
    }
  }
}

bool QuadrotorSim::LoadModelIntoViewer(mj::Simulate& sim, const fs::path& model_path, bool replace_existing) {
  sim.LoadMessage(model_path.string().c_str());

  const ModelLoadResult load_result = LoadModelFile(model_path);
  if (load_result.model == nullptr) {
    SetLoadError(sim, load_result.message);
    sim.LoadMessageClear();
    return false;
  }

  const std::string validation_error = ValidateModel(load_result.model);
  if (!validation_error.empty()) {
    mj_deleteModel(load_result.model);
    SetLoadError(sim, validation_error);
    sim.LoadMessageClear();
    return false;
  }

  ResolveCameraSensors(load_result.model, nullptr);
  ApplyDepthPluginPerformanceConfig(load_result.model);
  load_result.model->opt.timestep = config_.simulation.dt;
  mjData* new_data = mj_makeData(load_result.model);
  if (new_data == nullptr) {
    mj_deleteModel(load_result.model);
    SetLoadError(sim, "Failed to allocate mjData.");
    sim.LoadMessageClear();
    return false;
  }

  sim.Load(load_result.model, new_data, model_path.string().c_str());

  {
    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    InstallModelPointers(load_result.model, new_data, model_path, replace_existing);
  }

  if (!load_result.message.empty()) {
    SetLoadError(sim, load_result.message);
  } else {
    SetLoadError(sim, "");
  }
  if (load_result.pause_after_load) {
    sim.run = 0;
  }
  return true;
}

void QuadrotorSim::InstallModelPointers(mjModel* new_model, mjData* new_data, const fs::path& model_path, bool replace_existing) {
  if (new_model == nullptr || new_data == nullptr) {
    throw std::runtime_error("Cannot install null MuJoCo model/data.");
  }

  const std::string validation_error = ValidateModel(new_model);
  if (!validation_error.empty()) {
    throw std::runtime_error(validation_error);
  }

  mjModel* old_model = model_;
  mjData* old_data = data_;
  dynamic_obstacle_runtime_.Clear();

  new_model->opt.timestep = config_.simulation.dt;
  model_ = new_model;
  data_ = new_data;
  config_.model.scene_xml = fs::absolute(model_path);

  bindings_.Resolve(model_);
  ResolveCameraSensors(model_, data_);
  ConfigureDefaultCamera();
  RefreshCameraRendering();

  next_log_time_ = 0.0;
  control_step_count_ = 0;
  runtime_.Reset();
  active_instance_ = this;
  mjcb_control = &QuadrotorSim::ControlCallback;
  mj_forward(model_, data_);
  for (auto& stream : camera_streams_) {
    stream.next_render_time = 0.0;
    stream.sequence = 0;
  }

  // Initialize dynamic obstacle manager after model is ready
  InitializeDynamicObstacleManager();

  if (replace_existing) {
    if (old_data != nullptr && old_data != new_data) {
      mj_deleteData(old_data);
    }
    if (old_model != nullptr && old_model != new_model) {
      mj_deleteModel(old_model);
    }
  }
}

bool QuadrotorSim::IsDepthStreamRenderable(const CameraStreamRuntime& stream) const {
  if (stream.kind != CameraStreamKind::kDepth) {
    return false;
  }
  return stream.sensor_data_adr >= 0 && stream.sensor_data_size == stream.width * stream.height;
}

bool QuadrotorSim::HasRenderableDepthStream() const {
  for (const auto& stream : camera_streams_) {
    if (IsDepthStreamRenderable(stream)) {
      return true;
    }
  }
  return false;
}

bool QuadrotorSim::HasDueDepthStreamAfterStep(double next_sim_time) const {
  if (data_ == nullptr) {
    return false;
  }

  for (const auto& stream : camera_streams_) {
    if (!IsDepthStreamRenderable(stream)) {
      continue;
    }
    if (next_sim_time + 1e-9 >= stream.next_render_time) {
      return true;
    }
  }
  return false;
}

void QuadrotorSim::ConfigureDefaultCamera() {
  if (model_ == nullptr) {
    return;
  }

  const int track_camera_id = bindings_.track_camera_id();
  if (track_camera_id >= 0) {
    camera_.type = mjCAMERA_FIXED;
    camera_.fixedcamid = track_camera_id;
    camera_.trackbodyid = -1;
    if (viewer_) {
      viewer_->camera = 2 + track_camera_id;
    }
  }
}

void QuadrotorSim::ResolveCameraSensors(const mjModel* model, const mjData* data) {
  if (model == nullptr) {
    return;
  }

  std::unordered_map<std::string, std::size_t> color_stream_indices;
  const std::vector<int> publishable_camera_ids = CollectPublishableCameraIds(model, config_.simulation.track_camera_name);
  std::size_t next_camera_index = 0;

  for (auto& stream : camera_streams_) {
    stream.camera_id = -1;
    stream.sensor_id = -1;
    stream.sensor_data_adr = -1;
    stream.sensor_data_size = 0;
    stream.camera_name.clear();
    stream.sensor_name.clear();

    if (stream.kind != CameraStreamKind::kColor) {
      continue;
    }

    if (next_camera_index >= publishable_camera_ids.size()) {
      throw std::runtime_error("Model does not provide enough publishable cameras for configured camera streams.");
    }

    stream.camera_id = publishable_camera_ids[next_camera_index++];
    const char* camera_name = mj_id2name(model, mjOBJ_CAMERA, stream.camera_id);
    stream.camera_name = camera_name != nullptr ? camera_name : "";
    stream.width = model->cam_resolution[2 * stream.camera_id + 0];
    stream.height = model->cam_resolution[2 * stream.camera_id + 1];
    if (stream.width <= 0 || stream.height <= 0) {
      throw std::runtime_error("Camera stream '" + stream.name + "' resolved to MJCF camera '" + stream.camera_name +
                               "' without a valid resolution.");
    }
    color_stream_indices.emplace(stream.name, &stream - camera_streams_.data());
  }

  for (auto& stream : camera_streams_) {
    if (stream.kind != CameraStreamKind::kDepth) {
      continue;
    }

    const auto color_it = color_stream_indices.find(CameraStreamBaseName(stream.name));
    if (color_it == color_stream_indices.end()) {
      throw std::runtime_error("Depth stream '" + stream.name + "' does not have a matching color stream.");
    }

    const CameraStreamRuntime& color_stream = camera_streams_[color_it->second];
    stream.camera_id = color_stream.camera_id;
    stream.camera_name = color_stream.camera_name;
    stream.width = color_stream.width;
    stream.height = color_stream.height;

    const std::vector<int> sensor_ids = CollectPluginCameraSensorIds(model, stream.camera_id);
    if (sensor_ids.empty()) {
      throw std::runtime_error("Depth stream '" + stream.name + "' requires a MuJoCo plugin sensor bound to camera '" + stream.camera_name + "'.");
    }
    if (sensor_ids.size() > 1) {
      throw std::runtime_error("Depth stream '" + stream.name + "' expected exactly one MuJoCo plugin sensor on camera '" + stream.camera_name +
                               "', but found " + std::to_string(sensor_ids.size()) + ".");
    }

    stream.sensor_id = sensor_ids.front();
    const char* sensor_name = mj_id2name(model, mjOBJ_SENSOR, stream.sensor_id);
    stream.sensor_name = sensor_name != nullptr ? sensor_name : "";

    if (data == nullptr) {
      continue;
    }

    const std::optional<RayCasterDepthLayout> layout = ReadRayCasterDepthLayout(model, data, stream.sensor_id);
    if (!layout.has_value()) {
      throw std::runtime_error("Depth stream '" + stream.name + "' must bind to a MuJoCo plugin sensor.");
    }

    const int expected_size = stream.width * stream.height;
    if (layout->width > 0 && layout->height > 0 && (layout->width != stream.width || layout->height != stream.height)) {
      throw std::runtime_error("Depth stream '" + stream.name + "' resolution mismatch: config expects " + std::to_string(stream.width) + "x" +
                               std::to_string(stream.height) + " but MJCF sensor '" + stream.sensor_name + "' is " + std::to_string(layout->width) +
                               "x" + std::to_string(layout->height) + ".");
    }
    if (layout->data_point != 0) {
      throw std::runtime_error("Depth stream '" + stream.name + "' must expose depth as the first ray caster data block.");
    }
    if (layout->data_size != expected_size) {
      throw std::runtime_error("Depth stream '" + stream.name + "' expects " + std::to_string(expected_size) + " depth samples, but MJCF sensor '" +
                               stream.sensor_name + "' publishes " + std::to_string(layout->data_size) + ".");
    }

    stream.sensor_data_adr = model->sensor_adr[stream.sensor_id] + layout->data_point;
    stream.sensor_data_size = layout->data_size;
  }
}

void QuadrotorSim::ApplyDepthPluginPerformanceConfig(mjModel* model) const {
  if (model == nullptr) {
    return;
  }

  for (const auto& stream : camera_streams_) {
    if (stream.kind != CameraStreamKind::kDepth) {
      continue;
    }

    const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, stream.sensor_name.c_str());
    if (sensor_id < 0 || model->sensor_type[sensor_id] != mjSENS_PLUGIN) {
      continue;
    }

    const int plugin_instance = model->sensor_plugin[sensor_id];
    if (plugin_instance < 0 || plugin_instance >= model->nplugin) {
      continue;
    }

    const int requested_depth_samples = stream.width * stream.height;
    if (requested_depth_samples > model->sensor_dim[sensor_id]) {
      throw std::runtime_error("Depth stream '" + stream.name + "' requests " + std::to_string(stream.width) + "x" + std::to_string(stream.height) +
                               " samples, but MJCF sensor '" + stream.sensor_name + "' was compiled with capacity for only " +
                               std::to_string(model->sensor_dim[sensor_id]) +
                               " samples. Increase the MJCF ray-caster size before using this resolution.");
    }

    OverwritePluginTextConfig(model, plugin_instance, "size", std::to_string(stream.width) + " " + std::to_string(stream.height), stream.name);

    if (!stream.data_type.empty()) {
      const std::string depth_data_type = ResolveRayCasterDepthDataType(stream.data_type);
      OverwritePluginTextConfig(model, plugin_instance, "sensor_data_types", depth_data_type, stream.name);
    }

    const int step_update = ComputeStepUpdate(config_.simulation.dt, stream.compute_period_seconds);
    OverwritePluginIntConfig(model, plugin_instance, "n_step_update", step_update, stream.name);

    if (stream.worker_threads > 0) {
      std::cerr << "quadrotor warning: depth stream '" << stream.name << "' enables ray-caster worker_threads=" << stream.worker_threads
                << ", but the upstream multi-thread path is experimental.\n";
      OverwritePluginIntConfig(model, plugin_instance, "num_thread", stream.worker_threads, stream.name);
    }
  }
}

void QuadrotorSim::InitializeCameraRendering() {
  if (!HasColorCameraStreams(camera_streams_) || camera_rendering_ready_ || camera_rendering_failed_ || model_ == nullptr) {
    return;
  }

  int max_width = 1;
  int max_height = 1;
  for (const auto& stream : camera_streams_) {
    if (stream.kind != CameraStreamKind::kColor) {
      continue;
    }
    max_width = std::max(max_width, stream.width);
    max_height = std::max(max_height, stream.height);
  }

  std::string error;
  if (!camera_renderer_.Initialize(model_, max_width, max_height, &error)) {
    camera_rendering_failed_ = true;
    std::cerr << "quadrotor warning: camera rendering is unavailable: " << error << '\n';
    return;
  }
  camera_rendering_ready_ = true;
}

void QuadrotorSim::RefreshCameraRendering() {
  if (!camera_rendering_ready_ || !HasColorCameraStreams(camera_streams_) || model_ == nullptr) {
    return;
  }

  int max_width = 1;
  int max_height = 1;
  for (const auto& stream : camera_streams_) {
    if (stream.kind != CameraStreamKind::kColor) {
      continue;
    }
    max_width = std::max(max_width, stream.width);
    max_height = std::max(max_height, stream.height);
  }

  std::string error;
  if (!camera_renderer_.RefreshModel(model_, max_width, max_height, &error)) {
    camera_rendering_ready_ = false;
    camera_rendering_failed_ = true;
    std::cerr << "quadrotor warning: failed to refresh camera rendering after model reload: " << error << '\n';
  }
}

void QuadrotorSim::RenderCameraFramesIfNeeded() {
  if (model_ == nullptr || data_ == nullptr) {
    return;
  }

  for (auto& stream : camera_streams_) {
    if (stream.kind == CameraStreamKind::kDepth) {
      if (!IsDepthStreamRenderable(stream) || data_->time + 1e-9 < stream.next_render_time) {
        continue;
      }
    } else if (stream.camera_id < 0 || data_->time + 1e-9 < stream.next_render_time) {
      continue;
    }

    CameraFrame frame;
    frame.sim_time = data_->time;
    frame.width = static_cast<std::uint32_t>(stream.width);
    frame.height = static_cast<std::uint32_t>(stream.height);
    frame.sequence = ++stream.sequence;

    if (stream.kind == CameraStreamKind::kColor) {
      if (!camera_rendering_ready_) {
        continue;
      }

      frame.format = CameraFrameFormat::kRgb8;
      frame.step = static_cast<std::uint32_t>(stream.width * CameraFrameBytesPerPixel(frame.format));

      std::string error;
      if (!camera_renderer_.RenderRgb(model_, data_, stream.camera_id, stream.width, stream.height, &frame.data, &error)) {
        camera_rendering_ready_ = false;
        camera_rendering_failed_ = true;
        std::cerr << "quadrotor warning: camera rendering stopped: " << error << '\n';
        return;
      }
    } else {
      if (stream.sensor_data_adr < 0 || stream.sensor_data_size != stream.width * stream.height) {
        continue;
      }

      frame.format = CameraFrameFormat::kDepth32F;
      frame.step = static_cast<std::uint32_t>(stream.width * CameraFrameBytesPerPixel(frame.format));

      const std::size_t pixel_count = static_cast<std::size_t>(stream.width) * static_cast<std::size_t>(stream.height);
      frame.data.resize(pixel_count * sizeof(float));

      const mjtNum* depth_samples = data_->sensordata + stream.sensor_data_adr;
      float* depth_pixels = reinterpret_cast<float*>(frame.data.data());
      for (std::size_t i = 0; i < pixel_count; ++i) {
        const mjtNum value = depth_samples[i];
        depth_pixels[i] = std::isfinite(value) ? static_cast<float>(value) : 0.0f;
      }
    }

    WriteCameraFrame(stream.channel_name, frame);
    stream.next_render_time = data_->time + stream.period_seconds;
  }
}

std::string QuadrotorSim::ValidateModel(const mjModel* candidate) const {
  const std::string bindings_error = bindings_.ValidateModel(candidate);
  if (!bindings_error.empty()) {
    return bindings_error;
  }

  const std::vector<int> publishable_camera_ids = CollectPublishableCameraIds(candidate, config_.simulation.track_camera_name);
  std::unordered_map<std::string, int> color_camera_ids;
  std::size_t next_camera_index = 0;

  for (const auto& stream : camera_streams_) {
    if (stream.kind != CameraStreamKind::kColor) {
      continue;
    }
    if (next_camera_index >= publishable_camera_ids.size()) {
      return "Model is incompatible with the configured camera streams: it does not expose enough publishable MJCF cameras.";
    }
    const int camera_id = publishable_camera_ids[next_camera_index++];
    const int width = candidate->cam_resolution[2 * camera_id + 0];
    const int height = candidate->cam_resolution[2 * camera_id + 1];
    if (width <= 0 || height <= 0) {
      const char* camera_name = mj_id2name(candidate, mjOBJ_CAMERA, camera_id);
      return "Model camera '" + std::string(camera_name != nullptr ? camera_name : "<unnamed>") + "' must define a positive resolution.";
    }
    color_camera_ids.emplace(stream.name, camera_id);
  }

  for (const auto& stream : camera_streams_) {
    if (stream.kind != CameraStreamKind::kDepth) {
      continue;
    }

    const auto color_it = color_camera_ids.find(CameraStreamBaseName(stream.name));
    if (color_it == color_camera_ids.end()) {
      return "Depth stream '" + stream.name + "' does not have a matching color stream.";
    }

    const int camera_id = color_it->second;
    const std::vector<int> sensor_ids = CollectPluginCameraSensorIds(candidate, camera_id);
    if (sensor_ids.empty()) {
      const char* camera_name = mj_id2name(candidate, mjOBJ_CAMERA, camera_id);
      return "Depth stream '" + stream.name + "' requires a MuJoCo plugin sensor bound to camera '" +
             std::string(camera_name != nullptr ? camera_name : "<unnamed>") + "'.";
    }
    if (sensor_ids.size() > 1) {
      const char* camera_name = mj_id2name(candidate, mjOBJ_CAMERA, camera_id);
      return "Depth stream '" + stream.name + "' expected exactly one MuJoCo plugin sensor bound to camera '" +
             std::string(camera_name != nullptr ? camera_name : "<unnamed>") + "'.";
    }
  }

  return {};
}

int QuadrotorSim::ComputeControlDecimation() const {
  if (config_.simulation.dt <= 0.0) {
    throw std::runtime_error("simulation.dt must be positive.");
  }
  if (config_.controller.rate_hz <= 0.0) {
    throw std::runtime_error("controller.rate_hz must be positive.");
  }
  const double physics_rate_hz = 1.0 / config_.simulation.dt;
  const double raw_decimation = physics_rate_hz / config_.controller.rate_hz;
  const int decimation = static_cast<int>(std::lround(raw_decimation));

  if (decimation <= 0) {
    throw std::runtime_error("controller.rate_hz must not exceed the physics update rate.");
  }

  const double snapped_rate_hz = physics_rate_hz / static_cast<double>(decimation);
  if (std::abs(snapped_rate_hz - config_.controller.rate_hz) > kFrequencyTolerance) {
    std::ostringstream stream;
    stream << "controller.rate_hz=" << config_.controller.rate_hz << " is incompatible with simulation.dt=" << config_.simulation.dt
           << ". The ratio physics_rate/controller_rate must be an integer. "
           << "For this dt, valid rates include " << physics_rate_hz << ", " << physics_rate_hz / 2.0 << ", " << physics_rate_hz / 4.0 << ", ...";
    throw std::runtime_error(stream.str());
  }

  return decimation;
}

bool QuadrotorSim::ShouldContinueHeadless() const {
  if (model_ == nullptr || data_ == nullptr || stop_requested_.load()) {
    return false;
  }
  return config_.simulation.duration <= 0.0 || data_->time < config_.simulation.duration;
}

void QuadrotorSim::SleepToMatchRealtime(const std::chrono::high_resolution_clock::time_point& step_start, double simulated_seconds) const {
  const auto current_time = std::chrono::high_resolution_clock::now();
  const double elapsed_sec = std::chrono::duration<double>(current_time - step_start).count();
  const double time_until_next_step = simulated_seconds - elapsed_sec;
  if (time_until_next_step > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(time_until_next_step));
  }
}

void QuadrotorSim::HandleSigint(int signal) {
  if (signal != SIGINT) {
    return;
  }
  if (active_instance_ != nullptr) {
    active_instance_->stop_requested_.store(true);
  }
}

void QuadrotorSim::LogStateIfNeeded(const TelemetrySnapshot& snapshot) const {
  if (config_.simulation.print_interval <= 0.0 || snapshot.sim_time + 1e-9 < next_log_time_) {
    return;
  }

  next_log_time_ += config_.simulation.print_interval;
  std::cout << "[t=" << std::fixed << std::setprecision(3) << snapshot.sim_time << " s] "
            << "pos=" << VectorToString(snapshot.state.position) << " "
            << "goal=" << VectorToString(snapshot.goal_state.position) << " "
            << "source=" << snapshot.goal_source << " "
            << "motor_krpm=" << VectorToString(snapshot.motor_speed_krpm) << '\n';
}

void QuadrotorSim::InitializeDynamicObstacleManager() {
  dynamic_obstacle_runtime_.Initialize(config_.dynamic_obstacle, model_, data_, "[QuadrotorSim]");
}

bool QuadrotorSim::PrepareDynamicObstaclesForStep() {
  if (model_ == nullptr || data_ == nullptr) {
    return false;
  }

  const double next_sim_time = data_->time + model_->opt.timestep;
  return dynamic_obstacle_runtime_.PrepareForStep(next_sim_time, HasRenderableDepthStream(), HasDueDepthStreamAfterStep(next_sim_time));
}

void QuadrotorSim::PublishDynamicObstaclesSnapshotIfDue() {
  if (data_ == nullptr || !dynamic_obstacle_runtime_.PublishEnabled()) {
    return;
  }

  const double publish_rate_hz = dynamic_obstacle_runtime_.PublishRateHz();
  if (publish_rate_hz <= 0.0 || data_->time + 1e-9 < next_dynamic_obstacle_publish_time_) {
    return;
  }

  ausim::DynamicObstaclesSnapshot snapshot;
  if (!dynamic_obstacle_runtime_.BuildSnapshot(snapshot)) {
    return;
  }

  ausim::WriteDynamicObstaclesSnapshot(snapshot);
  next_dynamic_obstacle_publish_time_ = data_->time + (1.0 / publish_rate_hz);
}

}  // namespace quadrotor
