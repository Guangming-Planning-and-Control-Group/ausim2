#include "sim/quadrotor_sim.hpp"

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

#include <GLFW/glfw3.h>

#include "mujoco_simulate/glfw_adapter.h"
#include "mujoco_simulate/simulate.h"
#include "runtime/data_board_interface.hpp"

namespace fs = std::filesystem;

namespace quadrotor {

QuadrotorSim* QuadrotorSim::active_instance_ = nullptr;

namespace {
namespace mj = ::mujoco;

using Seconds = std::chrono::duration<double>;

constexpr double kSyncMisalign = 0.1;

void LoadMuJoCoPlugins() {
  const char* plugin_dir = std::getenv("MUJOCO_PLUGIN_DIR");
  if (plugin_dir) {
    mj_loadAllPluginLibraries(plugin_dir, nullptr);
  }
}

constexpr double kSimRefreshFraction = 0.7;
constexpr int kLoadErrorLength = 1024;
constexpr double kFrequencyTolerance = 1e-6;

std::string VectorToString(const Eigen::Vector4d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ", " << vector[3] << ")";
  return stream.str();
}

std::string VectorToString(const Eigen::Vector3d& vector) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "(" << vector.x() << ", " << vector.y() << ", " << vector.z() << ")";
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

QuadrotorSim::QuadrotorSim(QuadrotorConfig config)
    : config_(std::move(config)),
      runtime_(config_),
      bindings_(config_),
      actuator_writer_(config_.vehicle) {
  control_decimation_ = ComputeControlDecimation();

  if (config_.robot.count <= 0) {
    throw std::runtime_error("robot.count must be a positive integer.");
  }
  if (config_.robot.count != 1) {
    throw std::runtime_error(
        "robot.count > 1 is not implemented yet. Current runtime supports exactly one simulated vehicle.");
  }

  for (const SensorConfig& sensor : config_.sensors) {
    if (!sensor.enabled || sensor.type != "camera") {
      continue;
    }
    if (sensor.source_name.empty()) {
      throw std::runtime_error(
          "Camera sensor '" + sensor.name + "' must define source_name to match an MJCF camera.");
    }
    if (sensor.width <= 0 || sensor.height <= 0) {
      throw std::runtime_error(
          "Camera sensor '" + sensor.name + "' must define positive width and height.");
    }
    if (sensor.rate_hz <= 0.0) {
      throw std::runtime_error(
          "Camera sensor '" + sensor.name + "' must define a positive rate_hz.");
    }

    CameraSensorRuntime camera_sensor;
    camera_sensor.source_name = sensor.source_name;
    camera_sensor.width = sensor.width;
    camera_sensor.height = sensor.height;
    camera_sensor.period_seconds = 1.0 / sensor.rate_hz;
    camera_sensors_.push_back(std::move(camera_sensor));
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
  mj_step(model_, data_);
}

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
  const bool recompute_control = control_step_count_ == 0;
  const double control_dt = control_decimation_ * model->opt.timestep;
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
  WriteTelemetrySnapshot(snapshot);

  if (recompute_control) {
    LogStateIfNeeded(snapshot);
  }

  control_step_count_ = (control_step_count_ + 1) % control_decimation_;
}

void QuadrotorSim::ResetSimulation() {
  if (model_ == nullptr || data_ == nullptr) {
    return;
  }
  mj_resetData(model_, data_);
  mj_forward(model_, data_);
  next_log_time_ = 0.0;
  control_step_count_ = 0;
  runtime_.Reset();
  for (auto& sensor : camera_sensors_) {
    sensor.next_render_time = 0.0;
    sensor.sequence = 0;
  }
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
  std::cout << "Running MuJoCo simulation with official simulate viewer: "
            << config_.model.scene_xml << '\n';

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
          RenderCameraFramesIfNeeded();
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
          RenderCameraFramesIfNeeded();
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
  config_.model.scene_xml = fs::absolute(model_path);

  bindings_.Resolve(model_);
  ResolveCameraSensors(model_);
  ConfigureDefaultCamera();
  RefreshCameraRendering();

  next_log_time_ = 0.0;
  control_step_count_ = 0;
  runtime_.Reset();
  active_instance_ = this;
  mjcb_control = &QuadrotorSim::ControlCallback;
  mj_forward(model_, data_);
  for (auto& sensor : camera_sensors_) {
    sensor.next_render_time = 0.0;
    sensor.sequence = 0;
  }

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

void QuadrotorSim::ResolveCameraSensors(const mjModel* model) {
  for (auto& sensor : camera_sensors_) {
    sensor.camera_id = mj_name2id(model, mjOBJ_CAMERA, sensor.source_name.c_str());
  }
}

void QuadrotorSim::InitializeCameraRendering() {
  if (camera_sensors_.empty() || camera_rendering_ready_ || camera_rendering_failed_ || model_ == nullptr) {
    return;
  }

  int max_width = 1;
  int max_height = 1;
  for (const auto& sensor : camera_sensors_) {
    max_width = std::max(max_width, sensor.width);
    max_height = std::max(max_height, sensor.height);
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
  if (!camera_rendering_ready_ || model_ == nullptr) {
    return;
  }

  int max_width = 1;
  int max_height = 1;
  for (const auto& sensor : camera_sensors_) {
    max_width = std::max(max_width, sensor.width);
    max_height = std::max(max_height, sensor.height);
  }

  std::string error;
  if (!camera_renderer_.RefreshModel(model_, max_width, max_height, &error)) {
    camera_rendering_ready_ = false;
    camera_rendering_failed_ = true;
    std::cerr << "quadrotor warning: failed to refresh camera rendering after model reload: "
              << error << '\n';
  }
}

void QuadrotorSim::RenderCameraFramesIfNeeded() {
  if (!camera_rendering_ready_ || model_ == nullptr || data_ == nullptr) {
    return;
  }

  for (auto& sensor : camera_sensors_) {
    if (sensor.camera_id < 0 || data_->time + 1e-9 < sensor.next_render_time) {
      continue;
    }

    CameraFrame frame;
    frame.sim_time = data_->time;
    frame.width = static_cast<std::uint32_t>(sensor.width);
    frame.height = static_cast<std::uint32_t>(sensor.height);
    frame.step = static_cast<std::uint32_t>(sensor.width * 3);
    frame.sequence = ++sensor.sequence;

    std::string error;
    if (!camera_renderer_.RenderRgb(
            model_,
            data_,
            sensor.camera_id,
            sensor.width,
            sensor.height,
            &frame.data,
            &error)) {
      camera_rendering_ready_ = false;
      camera_rendering_failed_ = true;
      std::cerr << "quadrotor warning: camera rendering stopped: " << error << '\n';
      return;
    }

    WriteCameraFrame(sensor.source_name, frame);
    sensor.next_render_time = data_->time + sensor.period_seconds;
  }
}

std::string QuadrotorSim::ValidateModel(const mjModel* candidate) const {
  const std::string bindings_error = bindings_.ValidateModel(candidate);
  if (!bindings_error.empty()) {
    return bindings_error;
  }

  for (const auto& sensor : camera_sensors_) {
    if (mj_name2id(candidate, mjOBJ_CAMERA, sensor.source_name.c_str()) < 0) {
      return "Model is incompatible with the configured camera sensor: missing camera '" +
             sensor.source_name + "'";
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
  if (config_.simulation.example_mode < 0 || config_.simulation.example_mode > 2) {
    throw std::runtime_error("simulation.example_mode must be 0, 1, or 2.");
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

}  // namespace quadrotor
