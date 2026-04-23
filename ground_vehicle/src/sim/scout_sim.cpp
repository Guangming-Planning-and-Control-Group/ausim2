#include "sim/scout_sim.hpp"

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
#include <thread>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>

#include "mujoco-3.6.0/simulate/glfw_adapter.h"
#include "mujoco-3.6.0/simulate/simulate.h"
#include "runtime/data_board_interface.hpp"
#include "runtime/lidar_snapshot.hpp"

namespace fs = std::filesystem;

namespace ground_vehicle {

ScoutSim* ScoutSim::active_instance_ = nullptr;

namespace {
namespace mj = ::mujoco;

using Seconds = std::chrono::duration<double>;

constexpr double kSyncMisalign = 0.1;
constexpr double kSimRefreshFraction = 0.7;
constexpr int kLoadErrorLength = 1024;
constexpr int kHeadlessStepsPerCycle = 5;

#ifndef GROUND_VEHICLE_MUJOCO_PLUGIN_FALLBACK_DIR
  #define GROUND_VEHICLE_MUJOCO_PLUGIN_FALLBACK_DIR ""
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

struct ModelLoadResult {
  mjModel* model = nullptr;
  std::string message;
  bool pause_after_load = false;
};

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

  const fs::path fallback_decoder_dir(GROUND_VEHICLE_MUJOCO_PLUGIN_FALLBACK_DIR);
  const bool fallback_has_decoders = !fallback_decoder_dir.empty() && ContainsDecoderPlugin(fallback_decoder_dir);

  AppendPluginDirectory(plugin_directories, fallback_decoder_dir);
  AppendPluginDirectoriesFromEnv(plugin_directories, fallback_has_decoders);

  if (const auto bundled_plugin_dir = ResolveBundledPluginDirectory()) {
    AppendPluginDirectory(plugin_directories, *bundled_plugin_dir);
  }

  for (const fs::path& plugin_directory : plugin_directories) {
    mj_loadAllPluginLibraries(plugin_directory.string().c_str(), nullptr);
  }
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
  GLFWwindow* probe_window = glfwCreateWindow(64, 64, "scout-viewer-probe", nullptr, nullptr);
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

std::optional<std::string> ReadPluginAttribute(const mjModel* model, int plugin_instance, const char* attribute_name) {
  if (model == nullptr || plugin_instance < 0 || plugin_instance >= model->nplugin || attribute_name == nullptr) {
    return std::nullopt;
  }

  const mjpPlugin* plugin = mjp_getPluginAtSlot(model->plugin[plugin_instance]);
  if (plugin == nullptr) {
    return std::nullopt;
  }

  const char* attr = model->plugin_attr + model->plugin_attradr[plugin_instance];
  for (int i = 0; i < plugin->nattribute; ++i) {
    if (std::strcmp(plugin->attributes[i], attribute_name) == 0) {
      return std::string(attr);
    }
    attr += std::strlen(attr) + 1;
  }
  return std::nullopt;
}

bool ParseLidarSize(const std::string& text, int* h_ray_num, int* v_ray_num) {
  if (h_ray_num == nullptr || v_ray_num == nullptr) {
    return false;
  }

  std::istringstream stream(text);
  int width = 0;
  int height = 0;
  if (!(stream >> width >> height)) {
    return false;
  }
  *h_ray_num = width;
  *v_ray_num = height;
  return true;
}

bool ParseLidarRange(const std::string& text, float* range_min_m, float* range_max_m) {
  if (range_min_m == nullptr || range_max_m == nullptr) {
    return false;
  }

  std::istringstream stream(text);
  float range_min = 0.0f;
  float range_max = 0.0f;
  if (!(stream >> range_min >> range_max)) {
    return false;
  }
  *range_min_m = range_min;
  *range_max_m = range_max;
  return true;
}

bool ParseScalarDouble(const std::string& text, double* value) {
  if (value == nullptr) {
    return false;
  }

  std::istringstream stream(text);
  double parsed = 0.0;
  if (!(stream >> parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

Eigen::Vector3d ReadVector3(const mjData* data, const SensorBinding& binding) {
  if (!binding.resolved || binding.dim < 3) {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(data->sensordata[binding.adr + 0], data->sensordata[binding.adr + 1], data->sensordata[binding.adr + 2]);
}

Eigen::Quaterniond ReadQuaternion(const mjData* data, const SensorBinding& binding) {
  if (!binding.resolved || binding.dim < 4) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(data->sensordata[binding.adr + 0], data->sensordata[binding.adr + 1], data->sensordata[binding.adr + 2],
                            data->sensordata[binding.adr + 3]);
}

std::string VectorToString(const Eigen::Vector3d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << "(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
  return stream.str();
}

}  // namespace

ScoutSim::ScoutSim(ScoutConfig config)
    : config_(std::move(config)),
      controller_(config_.drive),
      actuator_writer_(config_.bindings.wheel_actuators),
      mode_machine_(config_.common.teleop_mode),
      gyro_sensor_{config_.common.state.gyro_sensor_name},
      accelerometer_sensor_{config_.common.state.accelerometer_sensor_name},
      quaternion_sensor_{config_.common.state.quaternion_sensor_name} {
  if (config_.common.robot.count <= 0) {
    throw std::runtime_error("robot.count must be a positive integer.");
  }
  if (config_.common.robot.count != 1) {
    throw std::runtime_error("robot.count > 1 is not implemented yet. Current scout runtime supports one vehicle.");
  }
  ausim::ClearVelocityCommand();
  ausim::ClearDiscreteCommand();
}

ScoutSim::~ScoutSim() {
  if (active_instance_ == this) {
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

void ScoutSim::LoadModel() {
  const ModelLoadResult load_result = LoadModelFile(config_.common.model.scene_xml);
  if (load_result.model == nullptr) {
    throw std::runtime_error("Failed to load model: " + load_result.message);
  }

  mjData* new_data = mj_makeData(load_result.model);
  if (new_data == nullptr) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error("Failed to allocate mjData.");
  }

  InstallModelPointers(load_result.model, new_data, config_.common.model.scene_xml, true);

  if (!load_result.message.empty()) {
    if (load_result.pause_after_load) {
      std::cout << "Model compiled, but simulation warning:\n  " << load_result.message << '\n';
    } else {
      std::cout << load_result.message << '\n';
    }
  }
}

void ScoutSim::InstallModelPointers(mjModel* new_model, mjData* new_data, const fs::path& model_path, bool replace_existing) {
  if (new_model == nullptr || new_data == nullptr) {
    throw std::runtime_error("Cannot install null MuJoCo model/data.");
  }

  const std::string actuator_error = actuator_writer_.ValidateModel(new_model);
  if (!actuator_error.empty()) {
    throw std::runtime_error(actuator_error);
  }

  mjModel* old_model = model_;
  mjData* old_data = data_;

  new_model->opt.timestep = config_.common.simulation.dt;
  model_ = new_model;
  data_ = new_data;
  config_.common.model.scene_xml = fs::absolute(model_path);

  ResolveBindings();
  ConfigureDefaultCamera();
  next_log_time_ = 0.0;
  last_wheel_speeds_ = WheelSpeeds{};
  last_command_source_ = "hold";
  last_command_valid_ = false;
  last_discrete_command_sequence_ = 0;
  last_discrete_command_status_ = ausim::DiscreteCommandAckStatus::kNone;
  mode_machine_.Reset();
  active_instance_ = this;
  mj_forward(model_, data_);
  InitializeDynamicObstacleManager();
  PublishTelemetry(false);

  if (replace_existing) {
    if (old_data != nullptr && old_data != new_data) {
      mj_deleteData(old_data);
    }
    if (old_model != nullptr && old_model != new_model) {
      mj_deleteModel(old_model);
    }
  }
}

void ScoutSim::ResolveBindings() {
  actuator_writer_.Resolve(model_);

  int freejoint_id = -1;
  if (!config_.bindings.freejoint_name.empty()) {
    freejoint_id = mj_name2id(model_, mjOBJ_JOINT, config_.bindings.freejoint_name.c_str());
  }
  if (freejoint_id < 0) {
    for (int joint_id = 0; joint_id < model_->njnt; ++joint_id) {
      if (model_->jnt_type[joint_id] == mjJNT_FREE) {
        freejoint_id = joint_id;
        break;
      }
    }
  }
  if (freejoint_id < 0 || model_->jnt_type[freejoint_id] != mjJNT_FREE) {
    throw std::runtime_error("Scout model must contain a freejoint for odometry.");
  }

  freejoint_qpos_adr_ = model_->jnt_qposadr[freejoint_id];
  freejoint_dof_adr_ = model_->jnt_dofadr[freejoint_id];

  ResolveSensor(&gyro_sensor_);
  ResolveSensor(&accelerometer_sensor_);
  ResolveSensor(&quaternion_sensor_);

  lidar_sensor_ = SensorBinding{};
  lidar_channel_name_.clear();
  lidar_h_ray_num_ = 0;
  lidar_v_ray_num_ = 0;
  lidar_fov_h_deg_ = 0.0f;
  lidar_fov_v_deg_ = 0.0f;
  lidar_range_min_m_ = 0.0f;
  lidar_range_max_m_ = 0.0f;

  for (const ausim::SensorConfig& sensor : config_.common.sensors) {
    if (sensor.type != "lidar" || !sensor.enabled) {
      continue;
    }

    lidar_channel_name_ = sensor.name;
    for (int sensor_id = 0; sensor_id < model_->nsensor; ++sensor_id) {
      if (model_->sensor_type[sensor_id] != mjSENS_PLUGIN) {
        continue;
      }

      const int plugin_instance = model_->sensor_plugin[sensor_id];
      const std::optional<std::string> size_attr = ReadPluginAttribute(model_, plugin_instance, "size");
      const std::optional<std::string> fov_h_attr = ReadPluginAttribute(model_, plugin_instance, "fov_h");
      const std::optional<std::string> fov_v_attr = ReadPluginAttribute(model_, plugin_instance, "fov_v");
      const std::optional<std::string> range_attr = ReadPluginAttribute(model_, plugin_instance, "dis_range");
      if (!size_attr.has_value() || !fov_h_attr.has_value() || !fov_v_attr.has_value() || !range_attr.has_value()) {
        continue;
      }

      lidar_sensor_.id = sensor_id;
      lidar_sensor_.adr = model_->sensor_adr[sensor_id];
      lidar_sensor_.dim = model_->sensor_dim[sensor_id];
      lidar_sensor_.resolved = true;
      const char* sensor_name = mj_id2name(model_, mjOBJ_SENSOR, sensor_id);
      lidar_sensor_.name = sensor_name != nullptr ? sensor_name : "";

      if (!ParseLidarSize(*size_attr, &lidar_h_ray_num_, &lidar_v_ray_num_)) {
        throw std::runtime_error("Scout lidar plugin attribute 'size' must be two integers.");
      }

      double fov_h = 0.0;
      double fov_v = 0.0;
      if (!ParseScalarDouble(*fov_h_attr, &fov_h) || !ParseScalarDouble(*fov_v_attr, &fov_v)) {
        throw std::runtime_error("Scout lidar plugin attributes 'fov_h' and 'fov_v' must be scalar numbers.");
      }
      lidar_fov_h_deg_ = static_cast<float>(fov_h);
      lidar_fov_v_deg_ = static_cast<float>(fov_v);
      if (!ParseLidarRange(*range_attr, &lidar_range_min_m_, &lidar_range_max_m_)) {
        throw std::runtime_error("Scout lidar plugin attribute 'dis_range' must be two scalar numbers.");
      }
      break;
    }

    if (!lidar_sensor_.resolved) {
      throw std::runtime_error("Scout lidar sensor '" + sensor.name + "' could not be resolved from the MJCF plugin definitions.");
    }
    if (lidar_h_ray_num_ <= 0 || lidar_v_ray_num_ <= 0) {
      throw std::runtime_error("Scout lidar plugin must define a positive 'size' attribute.");
    }
    if (lidar_sensor_.dim != lidar_h_ray_num_ * lidar_v_ray_num_) {
      throw std::runtime_error("Scout lidar plugin sample count does not match its configured size.");
    }
    break;
  }
}

void ScoutSim::ResolveSensor(SensorBinding* binding) const {
  binding->id = -1;
  binding->adr = -1;
  binding->dim = 0;
  binding->resolved = false;

  if (binding->name.empty()) {
    return;
  }

  const int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, binding->name.c_str());
  if (sensor_id < 0) {
    return;
  }

  binding->id = sensor_id;
  binding->adr = model_->sensor_adr[sensor_id];
  binding->dim = model_->sensor_dim[sensor_id];
  binding->resolved = true;
}

void ScoutSim::Step() {
  if (model_ == nullptr || data_ == nullptr) {
    throw std::runtime_error("Simulation is not loaded.");
  }

  ApplyControl();
  PrepareDynamicObstaclesForStep();
  mj_step(model_, data_);
  PublishTelemetry();
}

void ScoutSim::ApplyControl() {
  if (const std::optional<ausim::DiscreteCommand> discrete_command = ausim::ReadDiscreteCommand();
      discrete_command.has_value() && discrete_command->sequence != last_discrete_command_sequence_) {
    if (HandleDiscreteCommand(*discrete_command)) {
      last_discrete_command_status_ = ausim::DiscreteCommandAckStatus::kSuccess;
    } else {
      last_discrete_command_status_ = ausim::DiscreteCommandAckStatus::kUnsupported;
    }
    last_discrete_command_sequence_ = discrete_command->sequence;
    ausim::ClearDiscreteCommand();
  }

  const std::optional<ausim::VelocityCommand> command = ausim::ReadFreshVelocityCommand(config_.common.ros2.command_timeout);
  if (mode_machine_.enabled()) {
    const bool motion_active = command.has_value() && MotionCommandActive(*command);
    mode_machine_.UpdateConditions({motion_active});
  }

  double linear_x = 0.0;
  double angular_z = 0.0;
  if (command.has_value() && (!mode_machine_.enabled() || mode_machine_.AcceptsMotion())) {
    linear_x = command->linear.x();
    angular_z = command->angular.z();
    last_command_source_ = mode_machine_.enabled() ? "teleop:" + mode_machine_.Snapshot().sub_state : "ros2_cmd_vel";
    last_command_valid_ = true;
  } else {
    last_command_source_ = mode_machine_.enabled() && !mode_machine_.Snapshot().sub_state.empty() ? "teleop:" + mode_machine_.Snapshot().sub_state : "hold";
    last_command_valid_ = false;
  }

  last_wheel_speeds_ = controller_.Compute(linear_x, angular_z);
  actuator_writer_.Write(data_, last_wheel_speeds_);
}

bool ScoutSim::HandleDiscreteCommand(const ausim::DiscreteCommand& command) {
  switch (command.kind) {
    case ausim::DiscreteCommandKind::kResetSimulation:
      ResetSimulation();
      ausim::ClearVelocityCommand();
      return true;
    case ausim::DiscreteCommandKind::kGenericEvent:
      // Scout doesn't register any action callbacks yet, but string events are
      // forwarded to the state machine which handles them via transitions.
      return mode_machine_.HandleEvent(command.event_name);
    case ausim::DiscreteCommandKind::kNone:
      return false;
  }
  return false;
}

void ScoutSim::PublishTelemetry(bool log_state) {
  ausim::TelemetrySnapshot snapshot;
  snapshot.sim_time = data_->time;

  if (freejoint_qpos_adr_ >= 0 && freejoint_qpos_adr_ + 6 < model_->nq) {
    snapshot.state.position =
        Eigen::Vector3d(data_->qpos[freejoint_qpos_adr_ + 0], data_->qpos[freejoint_qpos_adr_ + 1], data_->qpos[freejoint_qpos_adr_ + 2]);
    snapshot.state.quaternion = Eigen::Quaterniond(data_->qpos[freejoint_qpos_adr_ + 3], data_->qpos[freejoint_qpos_adr_ + 4],
                                                   data_->qpos[freejoint_qpos_adr_ + 5], data_->qpos[freejoint_qpos_adr_ + 6]);
    snapshot.state.quaternion.normalize();
  }

  if (freejoint_dof_adr_ >= 0 && freejoint_dof_adr_ + 5 < model_->nv) {
    snapshot.state.velocity =
        Eigen::Vector3d(data_->qvel[freejoint_dof_adr_ + 0], data_->qvel[freejoint_dof_adr_ + 1], data_->qvel[freejoint_dof_adr_ + 2]);
    snapshot.state.omega =
        Eigen::Vector3d(data_->qvel[freejoint_dof_adr_ + 3], data_->qvel[freejoint_dof_adr_ + 4], data_->qvel[freejoint_dof_adr_ + 5]);
  }

  snapshot.imu.orientation = quaternion_sensor_.resolved ? ReadQuaternion(data_, quaternion_sensor_) : snapshot.state.quaternion;
  snapshot.imu.orientation.normalize();
  snapshot.imu.angular_velocity = gyro_sensor_.resolved ? ReadVector3(data_, gyro_sensor_) : snapshot.state.omega;
  if (accelerometer_sensor_.resolved) {
    snapshot.imu.linear_acceleration = ReadVector3(data_, accelerometer_sensor_);
    snapshot.imu.has_linear_acceleration = true;
  }

  snapshot.goal_source = last_command_source_;
  snapshot.has_goal = last_command_valid_;
  snapshot.robot_mode = mode_machine_.Snapshot();
  snapshot.last_discrete_command_sequence = last_discrete_command_sequence_;
  snapshot.last_discrete_command_status = last_discrete_command_status_;
  ausim::WriteTelemetrySnapshot(snapshot);
  if (log_state) {
    LogStateIfNeeded(snapshot);
  }

  PublishLidar();
}

void ScoutSim::PublishLidar() {
  if (!lidar_sensor_.resolved || lidar_sensor_.dim <= 0 || lidar_channel_name_.empty()) {
    return;
  }

  const int n = lidar_sensor_.dim;
  ausim::LidarSnapshot snap;
  snap.sim_time = data_->time;
  snap.h_ray_num = lidar_h_ray_num_;
  snap.v_ray_num = lidar_v_ray_num_;
  snap.fov_h_deg = lidar_fov_h_deg_;
  snap.fov_v_deg = lidar_fov_v_deg_;
  snap.range_min_m = lidar_range_min_m_;
  snap.range_max_m = lidar_range_max_m_;
  snap.ranges.resize(n);
  for (int i = 0; i < n; ++i) {
    snap.ranges[i] = static_cast<float>(data_->sensordata[lidar_sensor_.adr + i]);
  }

  ausim::WriteLidarSnapshot(lidar_channel_name_, snap);
}

void ScoutSim::LogStateIfNeeded(const ausim::TelemetrySnapshot& snapshot) const {
  if (config_.common.simulation.print_interval <= 0.0) {
    return;
  }
  if (snapshot.sim_time + 1e-9 < next_log_time_) {
    return;
  }

  const auto& speeds = last_wheel_speeds_.rad_per_second;
  std::ostringstream stream;
  stream << "t=" << std::fixed << std::setprecision(3) << snapshot.sim_time << " pos=" << VectorToString(snapshot.state.position)
         << " vel=" << VectorToString(snapshot.state.velocity) << " wheels(fr,fl,rl,rr)=(" << std::setprecision(2) << speeds[0] << ", " << speeds[1]
         << ", " << speeds[2] << ", " << speeds[3] << ")"
         << " source=" << snapshot.goal_source;
  std::cout << stream.str() << '\n';
  next_log_time_ = snapshot.sim_time + config_.common.simulation.print_interval;
}

void ScoutSim::ResetSimulation() {
  if (model_ == nullptr || data_ == nullptr) {
    return;
  }

  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  dynamic_obstacle_runtime_.ResetToCurrentTime();
  next_log_time_ = 0.0;
  last_wheel_speeds_ = WheelSpeeds{};
  last_command_source_ = "hold";
  last_command_valid_ = false;
  last_discrete_command_sequence_ = 0;
  last_discrete_command_status_ = ausim::DiscreteCommandAckStatus::kNone;
  mode_machine_.Reset();
}

void ScoutSim::InitializeDynamicObstacleManager() {
  dynamic_obstacle_runtime_.Initialize(config_.common.dynamic_obstacle, model_, data_, "[ScoutSim]");
}

bool ScoutSim::PrepareDynamicObstaclesForStep() {
  if (model_ == nullptr || data_ == nullptr) {
    return false;
  }
  return dynamic_obstacle_runtime_.PrepareForStep(
      data_->time + model_->opt.timestep,
      false,
      false);
}

void ScoutSim::Run() {
  stop_requested_.store(false);
  active_instance_ = this;
  InitializeVisualizationState();

  if (config_.common.viewer.enabled) {
    std::string viewer_error;
    if (!CanInitializeOfficialViewer(&viewer_error)) {
      if (!config_.common.viewer.fallback_to_headless) {
        throw std::runtime_error("Official simulate viewer unavailable: " + viewer_error);
      }
      std::cerr << "viewer unavailable, falling back to headless: " << viewer_error << '\n';
    } else {
      InitializeViewer();
    }
  }

  std::signal(SIGINT, &ScoutSim::HandleSigint);

  if (viewer_) {
    RunWithViewer();
  } else {
    if (model_ == nullptr || data_ == nullptr) {
      LoadModel();
    }
    std::cout << "Running Scout MuJoCo simulation with model: " << config_.common.model.scene_xml << '\n';
    std::cout << "Duration: " << config_.common.simulation.duration << " s, dt: " << config_.common.simulation.dt << " s\n";
    RunHeadless();
  }

  std::signal(SIGINT, SIG_DFL);
  active_instance_ = nullptr;

  if (data_ != nullptr) {
    std::cout << "Scout simulation finished at t=" << std::fixed << std::setprecision(3) << data_->time
              << " s, final position=" << VectorToString(Eigen::Vector3d(data_->qpos[0], data_->qpos[1], data_->qpos[2])) << '\n';
  }
}

void ScoutSim::RunHeadless() {
  while (ShouldContinue()) {
    const auto step_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kHeadlessStepsPerCycle && ShouldContinue(); ++i) {
      Step();
    }
    SleepToMatchRealtime(step_start);
  }
}

bool ScoutSim::MotionCommandActive(const ausim::VelocityCommand& command) {
  return std::abs(command.linear.x()) > 1e-3 || std::abs(command.angular.z()) > 1e-3;
}

void ScoutSim::RunWithViewer() {
  std::cout << "Running Scout MuJoCo simulation with official simulate viewer: " << config_.common.model.scene_xml << '\n';

  physics_thread_ = std::thread(&ScoutSim::PhysicsThreadMain, this);
  viewer_->RenderLoop();

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  viewer_.reset();
}

void ScoutSim::InitializeVisualizationState() {
  if (visualization_state_initialized_) {
    return;
  }

  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&visualization_options_);
  mjv_defaultPerturb(&perturbation_);
  visualization_state_initialized_ = true;
}

void ScoutSim::ConfigureDefaultCamera() {
  if (model_ == nullptr) {
    return;
  }

  mjv_defaultCamera(&camera_);
  camera_.type = mjCAMERA_FREE;
  camera_.lookat[0] = 0.0;
  camera_.lookat[1] = 0.0;
  camera_.lookat[2] = 0.25;
  camera_.distance = 3.0;
  camera_.azimuth = 135.0;
  camera_.elevation = -25.0;

  if (config_.common.simulation.track_camera_name.empty()) {
    return;
  }

  const int track_camera_id = mj_name2id(model_, mjOBJ_CAMERA, config_.common.simulation.track_camera_name.c_str());
  if (track_camera_id < 0) {
    std::cerr << "scout warning: configured track camera '" << config_.common.simulation.track_camera_name
              << "' was not found; using free camera instead.\n";
    return;
  }

  camera_.type = mjCAMERA_FIXED;
  camera_.fixedcamid = track_camera_id;
  camera_.trackbodyid = -1;
  if (viewer_) {
    viewer_->camera = 2 + track_camera_id;
  }
}

void ScoutSim::InitializeViewer() {
  if (viewer_) {
    return;
  }

  viewer_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &camera_, &visualization_options_, &perturbation_,
                                           /* is_passive = */ false);

  const int mjui_enabled = config_.common.viewer.mjui_enabled ? 1 : 0;
  viewer_->ui0_enable = mjui_enabled;
  viewer_->ui1_enable = mjui_enabled;
  if (!config_.common.viewer.mjui_enabled) {
    viewer_->help = 0;
    viewer_->info = 0;
    viewer_->profiler = 0;
    viewer_->sensor = 0;
  }

  viewer_->vsync = config_.common.viewer.vsync ? 1 : 0;
}

void ScoutSim::CleanupViewer() {
  if (viewer_) {
    viewer_->exitrequest.store(true);
  }

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  viewer_.reset();
}

void ScoutSim::PhysicsThreadMain() {
  try {
    if (viewer_ == nullptr) {
      return;
    }

    if (model_ != nullptr && data_ != nullptr) {
      const std::string displayed_filename = config_.common.model.scene_xml.string();
      viewer_->Load(model_, data_, displayed_filename.c_str());
      const std::unique_lock<std::recursive_mutex> lock(viewer_->mtx);
      mj_forward(model_, data_);
      SetLoadError(*viewer_, "");
    } else if (!LoadModelIntoViewer(*viewer_, config_.common.model.scene_xml, false)) {
      viewer_->exitrequest.store(true);
      return;
    }

    PhysicsLoop(*viewer_);
    viewer_->exitrequest.store(true);
  } catch (const std::exception& error) {
    if (viewer_) {
      SetLoadError(*viewer_, error.what());
      viewer_->exitrequest.store(true);
    }
  }
}

bool ScoutSim::LoadModelIntoViewer(mj::Simulate& sim, const fs::path& model_path, bool replace_existing) {
  sim.LoadMessage(model_path.string().c_str());

  const ModelLoadResult load_result = LoadModelFile(model_path);
  if (load_result.model == nullptr) {
    SetLoadError(sim, load_result.message);
    sim.LoadMessageClear();
    return false;
  }

  const std::string actuator_error = actuator_writer_.ValidateModel(load_result.model);
  if (!actuator_error.empty()) {
    mj_deleteModel(load_result.model);
    SetLoadError(sim, actuator_error);
    sim.LoadMessageClear();
    return false;
  }

  load_result.model->opt.timestep = config_.common.simulation.dt;
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

  if (load_result.pause_after_load && !load_result.message.empty()) {
    SetLoadError(sim, load_result.message);
  } else {
    SetLoadError(sim, "");
  }
  if (load_result.pause_after_load) {
    sim.run = 0;
  }
  return true;
}

void ScoutSim::PhysicsLoop(mj::Simulate& sim) {
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

    if (config_.common.simulation.duration > 0.0 && data_ != nullptr && data_->time >= config_.common.simulation.duration) {
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
        ApplyControl();
        PrepareDynamicObstaclesForStep();
        mj_step(model_, data_);
        PublishTelemetry();

        if (const char* message = Diverged(model_->opt.disableflags, data_)) {
          sim.run = 0;
          SetLoadError(sim, message);
        } else {
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
          ApplyControl();
          PrepareDynamicObstaclesForStep();
          mj_step(model_, data_);
          PublishTelemetry();

          if (const char* message = Diverged(model_->opt.disableflags, data_)) {
            sim.run = 0;
            SetLoadError(sim, message);
            break;
          }

          stepped = true;
          if (data_->time < previous_sim) {
            break;
          }
          if (config_.common.simulation.duration > 0.0 && data_->time >= config_.common.simulation.duration) {
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
      PublishTelemetry(false);
    }
  }
}

bool ScoutSim::ShouldContinue() const {
  if (stop_requested_.load()) {
    return false;
  }
  const double duration = config_.common.simulation.duration;
  return duration <= 0.0 || data_->time < duration;
}

void ScoutSim::SleepToMatchRealtime(const std::chrono::high_resolution_clock::time_point& step_start) const {
  const auto target = step_start + std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                                       std::chrono::duration<double>(model_->opt.timestep * kHeadlessStepsPerCycle));
  std::this_thread::sleep_until(target);
}

void ScoutSim::HandleSigint(int signal) {
  if (signal == SIGINT && active_instance_ != nullptr) {
    active_instance_->stop_requested_.store(true);
    if (active_instance_->viewer_ != nullptr) {
      active_instance_->viewer_->exitrequest.store(true);
    }
  }
}

}  // namespace ground_vehicle
