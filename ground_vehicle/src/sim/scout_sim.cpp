#include "sim/scout_sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "runtime/data_board_interface.hpp"

namespace fs = std::filesystem;

namespace ground_vehicle {

ScoutSim* ScoutSim::active_instance_ = nullptr;

namespace {

constexpr int kLoadErrorLength = 1024;

#ifndef GROUND_VEHICLE_MUJOCO_PLUGIN_FALLBACK_DIR
#define GROUND_VEHICLE_MUJOCO_PLUGIN_FALLBACK_DIR ""
#endif

struct ModelLoadResult {
  mjModel* model = nullptr;
  std::string message;
};

bool ContainsDecoderPlugin(const fs::path& directory) {
  std::error_code error;
  return fs::exists(directory / "libobj_decoder.so", error) ||
         fs::exists(directory / "libstl_decoder.so", error);
}

void AppendPluginDirectory(
    std::vector<fs::path>* directories,
    const fs::path& directory,
    bool skip_decoder_directory) {
  if (directories == nullptr || directory.empty()) {
    return;
  }

  std::error_code error;
  if (!fs::exists(directory, error) || !fs::is_directory(directory, error)) {
    return;
  }
  if (skip_decoder_directory && ContainsDecoderPlugin(directory)) {
    return;
  }
  for (const fs::path& existing : *directories) {
    if (existing == directory) {
      return;
    }
  }
  directories->push_back(directory);
}

void AppendPluginDirectoriesFromEnv(
    std::vector<fs::path>* directories,
    bool skip_decoder_directories) {
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
  const bool fallback_has_decoders =
      !fallback_decoder_dir.empty() && ContainsDecoderPlugin(fallback_decoder_dir);

  AppendPluginDirectory(&plugin_directories, fallback_decoder_dir, false);
  AppendPluginDirectoriesFromEnv(&plugin_directories, fallback_has_decoders);

  for (const fs::path& plugin_directory : plugin_directories) {
    mj_loadAllPluginLibraries(plugin_directory.string().c_str(), nullptr);
  }
}

ModelLoadResult LoadModelFile(const fs::path& path) {
  LoadMuJoCoPlugins();

  ModelLoadResult result;
  char load_error[kLoadErrorLength] = "Could not load binary model";

  if (path.extension() == ".mjb") {
    result.model = mj_loadModel(path.string().c_str(), nullptr);
    if (result.model == nullptr) {
      result.message = load_error;
    }
    return result;
  }

  result.model = mj_loadXML(path.string().c_str(), nullptr, load_error, sizeof(load_error));
  if (load_error[0] != '\0') {
    const std::size_t length = std::strlen(load_error);
    if (length > 0 && load_error[length - 1] == '\n') {
      load_error[length - 1] = '\0';
    }
  }
  if (result.model == nullptr) {
    result.message = load_error;
  }
  return result;
}

Eigen::Vector3d ReadVector3(const mjData* data, const SensorBinding& binding) {
  if (!binding.resolved || binding.dim < 3) {
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(
      data->sensordata[binding.adr + 0],
      data->sensordata[binding.adr + 1],
      data->sensordata[binding.adr + 2]);
}

Eigen::Quaterniond ReadQuaternion(const mjData* data, const SensorBinding& binding) {
  if (!binding.resolved || binding.dim < 4) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(
      data->sensordata[binding.adr + 0],
      data->sensordata[binding.adr + 1],
      data->sensordata[binding.adr + 2],
      data->sensordata[binding.adr + 3]);
}

std::string VectorToString(const Eigen::Vector3d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
  return stream.str();
}

}  // namespace

ScoutSim::ScoutSim(ScoutConfig config)
    : config_(std::move(config)),
      controller_(config_.drive),
      actuator_writer_(config_.bindings.wheel_actuators),
      gyro_sensor_{config_.common.state.gyro_sensor_name},
      accelerometer_sensor_{config_.common.state.accelerometer_sensor_name},
      quaternion_sensor_{config_.common.state.quaternion_sensor_name} {
  if (config_.common.robot.count <= 0) {
    throw std::runtime_error("robot.count must be a positive integer.");
  }
  if (config_.common.robot.count != 1) {
    throw std::runtime_error(
        "robot.count > 1 is not implemented yet. Current scout runtime supports one vehicle.");
  }
}

ScoutSim::~ScoutSim() {
  if (active_instance_ == this) {
    active_instance_ = nullptr;
  }
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

  const std::string actuator_error = actuator_writer_.ValidateModel(load_result.model);
  if (!actuator_error.empty()) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error(actuator_error);
  }

  load_result.model->opt.timestep = config_.common.simulation.dt;
  mjData* new_data = mj_makeData(load_result.model);
  if (new_data == nullptr) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error("Failed to allocate mjData.");
  }

  model_ = load_result.model;
  data_ = new_data;
  ResolveBindings();
  mj_forward(model_, data_);
  PublishTelemetry(false);
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
  mj_step(model_, data_);
  PublishTelemetry();
}

void ScoutSim::ApplyControl() {
  const std::optional<quadrotor::VelocityCommand> command =
      quadrotor::ReadFreshVelocityCommand(config_.common.ros2.command_timeout);

  double linear_x = 0.0;
  double angular_z = 0.0;
  if (command.has_value()) {
    linear_x = command->linear.x();
    angular_z = command->angular.z();
    last_command_source_ = "ros2_cmd_vel";
    last_command_valid_ = true;
  } else {
    last_command_source_ = "hold";
    last_command_valid_ = false;
  }

  last_wheel_speeds_ = controller_.Compute(linear_x, angular_z);
  actuator_writer_.Write(data_, last_wheel_speeds_);
}

void ScoutSim::PublishTelemetry(bool log_state) {
  quadrotor::TelemetrySnapshot snapshot;
  snapshot.sim_time = data_->time;

  if (freejoint_qpos_adr_ >= 0 && freejoint_qpos_adr_ + 6 < model_->nq) {
    snapshot.state.position = Eigen::Vector3d(
        data_->qpos[freejoint_qpos_adr_ + 0],
        data_->qpos[freejoint_qpos_adr_ + 1],
        data_->qpos[freejoint_qpos_adr_ + 2]);
    snapshot.state.quaternion = Eigen::Quaterniond(
        data_->qpos[freejoint_qpos_adr_ + 3],
        data_->qpos[freejoint_qpos_adr_ + 4],
        data_->qpos[freejoint_qpos_adr_ + 5],
        data_->qpos[freejoint_qpos_adr_ + 6]);
    snapshot.state.quaternion.normalize();
  }

  if (freejoint_dof_adr_ >= 0 && freejoint_dof_adr_ + 5 < model_->nv) {
    snapshot.state.velocity = Eigen::Vector3d(
        data_->qvel[freejoint_dof_adr_ + 0],
        data_->qvel[freejoint_dof_adr_ + 1],
        data_->qvel[freejoint_dof_adr_ + 2]);
    snapshot.state.omega = Eigen::Vector3d(
        data_->qvel[freejoint_dof_adr_ + 3],
        data_->qvel[freejoint_dof_adr_ + 4],
        data_->qvel[freejoint_dof_adr_ + 5]);
  }

  snapshot.imu.orientation =
      quaternion_sensor_.resolved ? ReadQuaternion(data_, quaternion_sensor_)
                                  : snapshot.state.quaternion;
  snapshot.imu.orientation.normalize();
  snapshot.imu.angular_velocity =
      gyro_sensor_.resolved ? ReadVector3(data_, gyro_sensor_) : snapshot.state.omega;
  if (accelerometer_sensor_.resolved) {
    snapshot.imu.linear_acceleration = ReadVector3(data_, accelerometer_sensor_);
    snapshot.imu.has_linear_acceleration = true;
  }

  snapshot.goal_source = last_command_source_;
  snapshot.has_goal = last_command_valid_;
  quadrotor::WriteTelemetrySnapshot(snapshot);
  if (log_state) {
    LogStateIfNeeded(snapshot);
  }
}

void ScoutSim::LogStateIfNeeded(const quadrotor::TelemetrySnapshot& snapshot) const {
  if (config_.common.simulation.print_interval <= 0.0) {
    return;
  }
  if (snapshot.sim_time + 1e-9 < next_log_time_) {
    return;
  }

  const auto& speeds = last_wheel_speeds_.rad_per_second;
  std::ostringstream stream;
  stream << "t=" << std::fixed << std::setprecision(3) << snapshot.sim_time
         << " pos=" << VectorToString(snapshot.state.position)
         << " vel=" << VectorToString(snapshot.state.velocity)
         << " wheels(fr,fl,rl,rr)=("
         << std::setprecision(2) << speeds[0] << ", " << speeds[1] << ", "
         << speeds[2] << ", " << speeds[3] << ")"
         << " source=" << snapshot.goal_source;
  std::cout << stream.str() << '\n';
  next_log_time_ = snapshot.sim_time + config_.common.simulation.print_interval;
}

void ScoutSim::Run() {
  stop_requested_.store(false);
  active_instance_ = this;
  std::signal(SIGINT, &ScoutSim::HandleSigint);

  std::cout << "Running Scout MuJoCo simulation with model: "
            << config_.common.model.scene_xml << '\n';
  std::cout << "Duration: " << config_.common.simulation.duration
            << " s, dt: " << config_.common.simulation.dt << " s\n";

  if (model_ == nullptr || data_ == nullptr) {
    LoadModel();
  }

  while (ShouldContinue()) {
    const auto step_start = std::chrono::high_resolution_clock::now();
    Step();
    SleepToMatchRealtime(step_start);
  }

  std::signal(SIGINT, SIG_DFL);
  active_instance_ = nullptr;

  std::cout << "Scout simulation finished at t=" << std::fixed << std::setprecision(3)
            << data_->time << " s, final position="
            << VectorToString(Eigen::Vector3d(data_->qpos[0], data_->qpos[1], data_->qpos[2]))
            << '\n';
}

bool ScoutSim::ShouldContinue() const {
  if (stop_requested_.load()) {
    return false;
  }
  const double duration = config_.common.simulation.duration;
  return duration <= 0.0 || data_->time < duration;
}

void ScoutSim::SleepToMatchRealtime(
    const std::chrono::high_resolution_clock::time_point& step_start) const {
  const auto target = step_start + std::chrono::duration_cast<
      std::chrono::high_resolution_clock::duration>(
      std::chrono::duration<double>(model_->opt.timestep));
  std::this_thread::sleep_until(target);
}

void ScoutSim::HandleSigint(int signal) {
  if (signal == SIGINT && active_instance_ != nullptr) {
    active_instance_->stop_requested_.store(true);
  }
}

}  // namespace ground_vehicle
