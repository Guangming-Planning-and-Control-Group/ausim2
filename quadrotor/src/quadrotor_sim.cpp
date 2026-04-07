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
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <GLFW/glfw3.h>
#include <yaml-cpp/yaml.h>

#include "mujoco_simulate/glfw_adapter.h"
#include "mujoco_simulate/simulate.h"

namespace fs = std::filesystem;

namespace quadrotor {

QuadrotorSim* QuadrotorSim::active_instance_ = nullptr;

namespace {
namespace mj = ::mujoco;

using Seconds = std::chrono::duration<double>;

constexpr double kSyncMisalign = 0.1;
constexpr double kSimRefreshFraction = 0.7;
constexpr int kLoadErrorLength = 1024;
constexpr double kFrequencyTolerance = 1e-6;

enum class ExampleMode {
  kSimple = 1,
  kCircular = 2,
};

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

std::string VectorToString(const Eigen::Vector3d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
  return stream.str();
}

std::string VectorToString(const Eigen::Vector4d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ", " << vector[3] << ")";
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

void SetLoadError(mj::Simulate& sim, const std::string& message) {
  std::snprintf(sim.load_error, sizeof(sim.load_error), "%s", message.c_str());
}

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
  ModelLoadResult result;

  char load_error[kLoadErrorLength] = "Could not load binary model";
  const auto load_start = mj::Simulate::Clock::now();

  if (path.extension() == ".mjb") {
    result.model = mj_loadModel(path.string().c_str(), nullptr);
    if (!result.model) {
      result.message = load_error;
      return result;
    }
  } else {
    result.model = mj_loadXML(path.string().c_str(), nullptr, load_error, sizeof(load_error));
    TrimTrailingNewline(load_error);
    if (!result.model) {
      result.message = load_error;
      return result;
    }
  }

  const double load_seconds =
      Seconds(mj::Simulate::Clock::now() - load_start).count();

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

QuadrotorConfig LoadConfigFromYaml(const std::string& path) {
  QuadrotorConfig config;

  const fs::path config_path = fs::absolute(path);
  if (!fs::exists(config_path)) {
    throw std::runtime_error("Config file does not exist: " + config_path.string());
  }

  const YAML::Node root = YAML::LoadFile(config_path.string());

  const YAML::Node model_node = root["model"];
  if (model_node && model_node["scene_xml"]) {
    config.simulation.model_path = ResolvePath(config_path, model_node["scene_xml"].as<std::string>());
  } else {
    config.simulation.model_path = ResolvePath(config_path, config.simulation.model_path);
  }

  const YAML::Node simulation_node = root["simulation"];
  AssignIfPresent(simulation_node, "duration", &config.simulation.duration);
  AssignIfPresent(simulation_node, "dt", &config.simulation.dt);
  AssignIfPresent(simulation_node, "print_interval", &config.simulation.print_interval);
  AssignIfPresent(simulation_node, "control_mode", &config.simulation.control_mode);
  AssignIfPresent(simulation_node, "example_mode", &config.simulation.example_mode);

  const YAML::Node vehicle_node = root["vehicle"];
  AssignIfPresent(vehicle_node, "mass", &config.vehicle.mass);
  AssignIfPresent(vehicle_node, "thrust_coefficient", &config.vehicle.Ct);
  AssignIfPresent(vehicle_node, "drag_coefficient", &config.vehicle.Cd);
  AssignIfPresent(vehicle_node, "arm_length", &config.vehicle.arm_length);
  AssignIfPresent(vehicle_node, "max_thrust", &config.vehicle.max_thrust);
  AssignIfPresent(vehicle_node, "max_torque", &config.vehicle.max_torque);
  AssignIfPresent(vehicle_node, "max_speed_krpm", &config.vehicle.max_speed_krpm);

  const YAML::Node controller_node = root["controller"];
  AssignIfPresent(controller_node, "kx", &config.controller.kx);
  AssignIfPresent(controller_node, "kv", &config.controller.kv);
  AssignIfPresent(controller_node, "kR", &config.controller.kR);
  AssignIfPresent(controller_node, "kw", &config.controller.kw);
  AssignIfPresent(controller_node, "rate_hz", &config.controller.rate_hz);
  AssignIfPresent(controller_node, "torque_scale", &config.torque_scale);

  const YAML::Node goal_node = root["goal"];
  if (goal_node) {
    config.hover_goal.position = LoadVector3(goal_node["position"], config.hover_goal.position);
    config.hover_goal.velocity = LoadVector3(goal_node["velocity"], config.hover_goal.velocity);
    config.hover_goal.heading = LoadVector3(goal_node["heading"], config.hover_goal.heading);
  }

  const YAML::Node trajectory_node = root["trajectory"];
  AssignIfPresent(trajectory_node, "wait_time", &config.circle_trajectory.wait_time);
  AssignIfPresent(trajectory_node, "height", &config.circle_trajectory.height);
  AssignIfPresent(trajectory_node, "radius", &config.circle_trajectory.radius);
  AssignIfPresent(trajectory_node, "speed_hz", &config.circle_trajectory.speed_hz);
  AssignIfPresent(trajectory_node, "height_gain", &config.circle_trajectory.height_gain);

  const YAML::Node viewer_node = root["viewer"];
  AssignIfPresent(viewer_node, "enabled", &config.viewer.enabled);
  AssignIfPresent(viewer_node, "fallback_to_headless", &config.viewer.fallback_to_headless);
  AssignIfPresent(viewer_node, "mjui_enabled", &config.viewer.mjui_enabled);
  AssignIfPresent(viewer_node, "vsync", &config.viewer.vsync);

  return config;
}

QuadrotorSim::QuadrotorSim(QuadrotorConfig config)
    : config_(std::move(config)),
      mixer_(MixerParams{
          config_.vehicle.Ct,
          config_.vehicle.Cd,
          config_.vehicle.arm_length,
          config_.vehicle.max_thrust,
          config_.vehicle.max_torque,
          config_.vehicle.max_speed_krpm}) {
  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&visualization_options_);
  mjv_defaultPerturb(&perturbation_);

  controller_.kx = config_.controller.kx;
  controller_.kv = config_.controller.kv;
  controller_.kR = config_.controller.kR;
  controller_.kw = config_.controller.kw;
  controller_.control_mode =
      static_cast<SE3Controller::ControlMode>(config_.simulation.control_mode);
  control_decimation_ = ComputeControlDecimation();
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
  const ModelLoadResult load_result = LoadModelFile(config_.simulation.model_path);
  if (load_result.model == nullptr) {
    throw std::runtime_error("Failed to load model: " + load_result.message);
  }

  const std::string validation_error = ValidateModel(load_result.model);
  if (!validation_error.empty()) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error(validation_error);
  }

  load_result.model->opt.timestep = config_.simulation.dt;
  mjData* new_data = mj_makeData(load_result.model);
  if (new_data == nullptr) {
    mj_deleteModel(load_result.model);
    throw std::runtime_error("Failed to allocate mjData.");
  }

  InstallModelPointers(load_result.model, new_data, config_.simulation.model_path, true);

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
  mj_step(model_, data_);
}

void QuadrotorSim::Run() {
  stop_requested_.store(false);

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
    std::cout << "Running MuJoCo simulation with model: " << config_.simulation.model_path << '\n';
    std::cout << "Duration: " << config_.simulation.duration << " s, dt: " << config_.simulation.dt << " s\n";
    RunHeadless();
  }

  std::signal(SIGINT, SIG_DFL);

  if (data_ != nullptr) {
    const State final_state = ReadCurrentState(data_);
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
  (void)model;
  std::optional<State> current_state;
  std::optional<State> goal_state;

  if (control_step_count_ == 0) {
    current_state = ReadCurrentState(data);
    Eigen::Vector3d forward = config_.hover_goal.heading;
    goal_state = BuildGoalState(data->time, *current_state, &forward);
    const bool use_position_hold =
        config_.simulation.control_mode == static_cast<int>(SE3Controller::ControlMode::kVelocity) &&
        data->time < config_.circle_trajectory.wait_time;
    controller_.control_mode = use_position_hold
                                   ? SE3Controller::ControlMode::kPosition
                                   : static_cast<SE3Controller::ControlMode>(config_.simulation.control_mode);
    const double control_dt = control_decimation_ * model->opt.timestep;
    cached_command_ = controller_.controlUpdate(*current_state, *goal_state, control_dt, forward);
  }

  const double gravity_magnitude = std::sqrt(
      model->opt.gravity[0] * model->opt.gravity[0] +
      model->opt.gravity[1] * model->opt.gravity[1] +
      model->opt.gravity[2] * model->opt.gravity[2]);
  const double mixer_thrust = cached_command_.thrust * gravity_magnitude * config_.vehicle.mass;
  const Eigen::Vector3d mixer_torque = cached_command_.angular * config_.torque_scale;
  const Eigen::Vector4d motor_speed =
      mixer_.calculate(mixer_thrust, mixer_torque.x(), mixer_torque.y(), mixer_torque.z());

  for (std::size_t i = 0; i < actuator_ids_.size(); ++i) {
    data->ctrl[actuator_ids_[i]] = CalcMotorInput(motor_speed[static_cast<Eigen::Index>(i)]);
  }

  if (current_state.has_value() && goal_state.has_value()) {
    LogStateIfNeeded(data, *current_state, *goal_state, motor_speed);
  }

  control_step_count_ = (control_step_count_ + 1) % control_decimation_;
}

State QuadrotorSim::ReadCurrentState(const mjData* data) const {
  State state;

  if (model_ != nullptr && model_->nq >= 3) {
    state.position = Eigen::Vector3d(data->qpos[0], data->qpos[1], data->qpos[2]);
  }
  if (model_ != nullptr && model_->nv >= 3) {
    state.velocity = Eigen::Vector3d(data->qvel[0], data->qvel[1], data->qvel[2]);
  }
  if (model_ != nullptr && model_->nsensordata >= 10) {
    state.omega = Eigen::Vector3d(data->sensordata[0], data->sensordata[1], data->sensordata[2]);
    state.quaternion = Eigen::Quaterniond(
        data->sensordata[6], data->sensordata[7], data->sensordata[8], data->sensordata[9]);
  } else {
    if (model_ != nullptr && model_->nv >= 6) {
      state.omega = Eigen::Vector3d(data->qvel[3], data->qvel[4], data->qvel[5]);
    }
    if (model_ != nullptr && model_->nq >= 7) {
      state.quaternion = Eigen::Quaterniond(data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]);
    }
  }

  state.quaternion.normalize();
  return state;
}

State QuadrotorSim::BuildGoalState(double time, const State& current, Eigen::Vector3d* forward) const {
  State goal_state;
  goal_state.quaternion = Eigen::Quaterniond::Identity();
  *forward = config_.hover_goal.heading;

  const bool velocity_mode =
      config_.simulation.control_mode == static_cast<int>(SE3Controller::ControlMode::kVelocity);
  const CircleTrajectoryConfig& circle = config_.circle_trajectory;
  if (velocity_mode && time < circle.wait_time) {
    goal_state.position = config_.hover_goal.position;
    goal_state.velocity = Eigen::Vector3d::Zero();
    return goal_state;
  }

  const auto example_mode = static_cast<ExampleMode>(config_.simulation.example_mode);
  if (example_mode == ExampleMode::kSimple) {
    goal_state.position = config_.hover_goal.position;
    goal_state.velocity = velocity_mode ? config_.hover_goal.velocity : Eigen::Vector3d::Zero();
    if (goal_state.velocity.norm() > 1e-6) {
      *forward = goal_state.velocity.normalized();
    }
    return goal_state;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double phase = 2.0 * kPi * circle.speed_hz * (time - circle.wait_time);
  const double cosine = std::cos(phase);
  const double sine = std::sin(phase);

  goal_state.position = Eigen::Vector3d(circle.radius * cosine, circle.radius * sine, circle.height);
  *forward = Eigen::Vector3d(-sine, cosine, 0.0);

  if (velocity_mode) {
    const Eigen::Vector2d base_xy(config_.hover_goal.velocity.x(), config_.hover_goal.velocity.y());
    const Eigen::Matrix2d rotation =
        (Eigen::Matrix2d() << cosine, -sine, sine, cosine).finished();
    const Eigen::Vector2d rotated_xy = rotation * base_xy;
    const double vertical_velocity =
        config_.hover_goal.velocity.z() +
        circle.height_gain * (config_.hover_goal.position.z() - current.position.z());

    goal_state.velocity = Eigen::Vector3d(rotated_xy.x(), rotated_xy.y(), vertical_velocity);
    if (rotated_xy.norm() > 1e-6) {
      *forward = Eigen::Vector3d(rotated_xy.x(), rotated_xy.y(), 0.0).normalized();
    }
  } else {
    goal_state.velocity = Eigen::Vector3d::Zero();
  }

  return goal_state;
}

void QuadrotorSim::ResolveActuatorIds() {
  static constexpr const char* kMotorNames[4] = {"motor1", "motor2", "motor3", "motor4"};
  for (std::size_t i = 0; i < actuator_ids_.size(); ++i) {
    actuator_ids_[i] = mj_name2id(model_, mjOBJ_ACTUATOR, kMotorNames[i]);
    if (actuator_ids_[i] < 0) {
      throw std::runtime_error(std::string("Failed to find actuator: ") + kMotorNames[i]);
    }
  }
}

double QuadrotorSim::CalcMotorForce(double krpm) const {
  return config_.vehicle.Ct * krpm * krpm;
}

double QuadrotorSim::CalcMotorInput(double krpm) const {
  const double clamped_speed = std::clamp(krpm, 0.0, config_.vehicle.max_speed_krpm);
  const double force = CalcMotorForce(clamped_speed);
  const double input = force / config_.vehicle.max_thrust;
  return std::clamp(input, 0.0, 1.0);
}

void QuadrotorSim::ResetSimulation() {
  if (model_ == nullptr || data_ == nullptr) {
    return;
  }
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  next_log_time_ = 0.0;
  control_step_count_ = 0;
  cached_command_ = ControlCommand{};
}

void QuadrotorSim::RunHeadless() {
  constexpr int kStepsPerCycle = 5;

  while (ShouldContinueHeadless()) {
    const auto step_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < kStepsPerCycle && ShouldContinueHeadless(); ++i) {
      Step();
    }
    SleepToMatchRealtime(step_start, model_->opt.timestep * kStepsPerCycle);
  }
}

void QuadrotorSim::RunWithViewer() {
  std::cout << "Running MuJoCo simulation with official simulate viewer: "
            << config_.simulation.model_path << '\n';

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

  viewer_ = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &camera_,
      &visualization_options_,
      &perturbation_,
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
      const std::string displayed_filename = config_.simulation.model_path.string();
      viewer_->Load(model_, data_, displayed_filename.c_str());
      const std::unique_lock<std::recursive_mutex> lock(viewer_->mtx);
      mj_forward(model_, data_);
      SetLoadError(*viewer_, "");
    } else if (!LoadModelIntoViewer(*viewer_, config_.simulation.model_path, false)) {
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

      const bool misaligned =
          std::abs(Seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > kSyncMisalign;

      if (elapsed_sim < 0.0 ||
          elapsed_cpu.count() < 0 ||
          sync_cpu.time_since_epoch().count() == 0 ||
          misaligned ||
          sim.speed_changed) {
        sync_cpu = start_cpu;
        sync_sim = data_->time;
        sim.speed_changed = false;

        sim.InjectNoise(sim.key);
        mj_step(model_, data_);

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
          mj_step(model_, data_);

          if (const char* message = Diverged(model_->opt.disableflags, data_)) {
            sim.run = 0;
            SetLoadError(sim, message);
            break;
          }

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
    }
  }
}

bool QuadrotorSim::LoadModelIntoViewer(
    mj::Simulate& sim,
    const fs::path& model_path,
    bool replace_existing) {
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

void QuadrotorSim::InstallModelPointers(
    mjModel* new_model,
    mjData* new_data,
    const fs::path& model_path,
    bool replace_existing) {
  if (new_model == nullptr || new_data == nullptr) {
    throw std::runtime_error("Cannot install null MuJoCo model/data.");
  }

  const std::string validation_error = ValidateModel(new_model);
  if (!validation_error.empty()) {
    throw std::runtime_error(validation_error);
  }

  mjModel* old_model = model_;
  mjData* old_data = data_;

  new_model->opt.timestep = config_.simulation.dt;
  model_ = new_model;
  data_ = new_data;
  config_.simulation.model_path = fs::absolute(model_path);

  ResolveActuatorIds();
  vehicle_body_id_ = mj_name2id(model_, mjOBJ_BODY, "cf2");
  if (vehicle_body_id_ < 0 && model_->nbody > 1) {
    vehicle_body_id_ = 1;
  }
  ConfigureDefaultCamera();

  next_log_time_ = 0.0;
  control_step_count_ = 0;
  cached_command_ = ControlCommand{};
  active_instance_ = this;
  mjcb_control = &QuadrotorSim::ControlCallback;
  mj_forward(model_, data_);

  if (replace_existing) {
    if (old_data != nullptr && old_data != new_data) {
      mj_deleteData(old_data);
    }
    if (old_model != nullptr && old_model != new_model) {
      mj_deleteModel(old_model);
    }
  }
}

void QuadrotorSim::ConfigureDefaultCamera() {
  if (model_ == nullptr) {
    return;
  }

  const int track_camera_id = mj_name2id(model_, mjOBJ_CAMERA, "track");
  if (track_camera_id >= 0) {
    camera_.type = mjCAMERA_FIXED;
    camera_.fixedcamid = track_camera_id;
    camera_.trackbodyid = -1;
    if (viewer_) {
      viewer_->camera = 2 + track_camera_id;
    }
  }
}

std::string QuadrotorSim::ValidateModel(const mjModel* candidate) const {
  static constexpr const char* kMotorNames[4] = {"motor1", "motor2", "motor3", "motor4"};
  for (const char* motor_name : kMotorNames) {
    if (mj_name2id(candidate, mjOBJ_ACTUATOR, motor_name) < 0) {
      return std::string("Model is incompatible with the quadrotor controller: missing actuator '") +
             motor_name + "'";
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
  if (config_.simulation.control_mode < 0 || config_.simulation.control_mode > 2) {
    throw std::runtime_error("simulation.control_mode must be 0, 1, or 2.");
  }
  if (config_.simulation.example_mode < 1 || config_.simulation.example_mode > 2) {
    throw std::runtime_error("simulation.example_mode must be 1 or 2.");
  }
  if (config_.simulation.control_mode == static_cast<int>(SE3Controller::ControlMode::kDirect)) {
    throw std::runtime_error(
        "simulation.control_mode=0 is reserved for direct thrust/motor control and is not implemented yet.");
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
    stream << "controller.rate_hz=" << config_.controller.rate_hz
           << " is incompatible with simulation.dt=" << config_.simulation.dt
           << ". The ratio physics_rate/controller_rate must be an integer. "
           << "For this dt, valid rates include " << physics_rate_hz << ", "
           << physics_rate_hz / 2.0 << ", " << physics_rate_hz / 4.0 << ", ...";
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

void QuadrotorSim::SleepToMatchRealtime(
    const std::chrono::high_resolution_clock::time_point& step_start,
    double simulated_seconds) const {
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

void QuadrotorSim::LogStateIfNeeded(
    const mjData* data,
    const State& current,
    const State& goal,
    const Eigen::Vector4d& motor_speed) const {
  if (config_.simulation.print_interval <= 0.0 || data->time + 1e-9 < next_log_time_) {
    return;
  }

  next_log_time_ += config_.simulation.print_interval;
  std::cout << "[t=" << std::fixed << std::setprecision(3) << data->time << " s] "
            << "pos=" << VectorToString(current.position) << " "
            << "goal=" << VectorToString(goal.position) << " "
            << "motor_krpm=" << VectorToString(motor_speed) << '\n';
}

}  // namespace quadrotor
