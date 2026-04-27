#include "manager/ros_bridge_process_manager.hpp"

#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "converts/ipc/bridge_packets.hpp"
#include "ipc/lidar_packet.hpp"
#include "ipc/socket_packet.hpp"
#include "ipc/socket_stream.hpp"
#include "runtime/data_board_interface.hpp"

namespace ausim {
namespace {

constexpr std::chrono::milliseconds kStartupProbeDelay(300);
constexpr std::chrono::milliseconds kShutdownPollDelay(50);
constexpr int kShutdownPollCount = 20;
constexpr std::size_t kMaxBridgeMessageSize = 64 * 1024;

std::runtime_error SystemError(const std::string& message) { return std::runtime_error(message + ": " + std::strerror(errno)); }

std::vector<std::uint8_t> BuildTaggedMessage(ipc::BridgeMessageType type, const void* payload, std::size_t payload_size) {
  std::vector<std::uint8_t> message(1 + payload_size);
  message[0] = static_cast<std::uint8_t>(type);
  if (payload_size > 0) {
    std::memcpy(message.data() + 1, payload, payload_size);
  }
  return message;
}

}  // namespace

RosBridgeProcessManager::RosBridgeProcessManager(const QuadrotorConfig& config, RosBridgeLaunchConfig launch_config)
    : config_(config), launch_config_(std::move(launch_config)) {
  for (const CameraStreamConfig& stream : BuildCameraStreamConfigs(config_.sensors)) {
    if (stream.channel_name.empty()) {
      throw std::runtime_error("Camera stream '" + stream.name + "' must define an internal channel name.");
    }
    camera_channel_names_.push_back(stream.channel_name);
  }
}

RosBridgeProcessManager::~RosBridgeProcessManager() { Stop(); }

void RosBridgeProcessManager::Start() {
  if (running_.load()) {
    return;
  }
  if (launch_config_.executable_path.empty()) {
    throw std::runtime_error("ROS bridge executable path is empty.");
  }
  if (!std::filesystem::exists(launch_config_.executable_path)) {
    throw std::runtime_error("ROS bridge executable does not exist: " + launch_config_.executable_path.string());
  }

  int telemetry_fds[2] = {-1, -1};
  int command_fds[2] = {-1, -1};
  int discrete_command_fds[2] = {-1, -1};
  int image_fds[2] = {-1, -1};
  int lidar_fds[2] = {-1, -1};
  if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, telemetry_fds) != 0) {
    throw SystemError("Failed to create telemetry socketpair");
  }
  if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, command_fds) != 0) {
    ipc::ShutdownAndClose(&telemetry_fds[0]);
    ipc::ShutdownAndClose(&telemetry_fds[1]);
    throw SystemError("Failed to create command socketpair");
  }
  if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, discrete_command_fds) != 0) {
    ipc::ShutdownAndClose(&telemetry_fds[0]);
    ipc::ShutdownAndClose(&telemetry_fds[1]);
    ipc::ShutdownAndClose(&command_fds[0]);
    ipc::ShutdownAndClose(&command_fds[1]);
    throw SystemError("Failed to create discrete command socketpair");
  }
  if (socketpair(AF_UNIX, SOCK_STREAM, 0, image_fds) != 0) {
    ipc::ShutdownAndClose(&telemetry_fds[0]);
    ipc::ShutdownAndClose(&telemetry_fds[1]);
    ipc::ShutdownAndClose(&command_fds[0]);
    ipc::ShutdownAndClose(&command_fds[1]);
    ipc::ShutdownAndClose(&discrete_command_fds[0]);
    ipc::ShutdownAndClose(&discrete_command_fds[1]);
    throw SystemError("Failed to create image socketpair");
  }
  if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, lidar_fds) != 0) {
    ipc::ShutdownAndClose(&telemetry_fds[0]);
    ipc::ShutdownAndClose(&telemetry_fds[1]);
    ipc::ShutdownAndClose(&command_fds[0]);
    ipc::ShutdownAndClose(&command_fds[1]);
    ipc::ShutdownAndClose(&discrete_command_fds[0]);
    ipc::ShutdownAndClose(&discrete_command_fds[1]);
    ipc::ShutdownAndClose(&image_fds[0]);
    ipc::ShutdownAndClose(&image_fds[1]);
    throw SystemError("Failed to create lidar socketpair");
  }

  telemetry_send_fd_ = telemetry_fds[0];
  command_recv_fd_ = command_fds[0];
  discrete_command_recv_fd_ = discrete_command_fds[0];
  image_send_fd_ = image_fds[0];
  lidar_send_fd_ = lidar_fds[0];
  ipc::SetNonBlocking(telemetry_send_fd_);

  child_pid_ = fork();
  if (child_pid_ < 0) {
    ipc::ShutdownAndClose(&telemetry_fds[0]);
    ipc::ShutdownAndClose(&telemetry_fds[1]);
    ipc::ShutdownAndClose(&command_fds[0]);
    ipc::ShutdownAndClose(&command_fds[1]);
    ipc::ShutdownAndClose(&discrete_command_fds[0]);
    ipc::ShutdownAndClose(&discrete_command_fds[1]);
    ipc::ShutdownAndClose(&image_fds[0]);
    ipc::ShutdownAndClose(&image_fds[1]);
    telemetry_send_fd_ = -1;
    command_recv_fd_ = -1;
    discrete_command_recv_fd_ = -1;
    image_send_fd_ = -1;
    throw SystemError("Failed to fork ROS bridge process");
  }

  if (child_pid_ == 0) {
    close(telemetry_fds[0]);
    close(command_fds[0]);
    close(discrete_command_fds[0]);
    close(image_fds[0]);
    close(lidar_fds[0]);

    std::vector<std::string> args;
    args.push_back(launch_config_.executable_path.string());
    args.insert(args.end(), launch_config_.config_arguments.begin(), launch_config_.config_arguments.end());
    args.push_back("--telemetry-fd");
    args.push_back(std::to_string(telemetry_fds[1]));
    args.push_back("--command-fd");
    args.push_back(std::to_string(command_fds[1]));
    args.push_back("--discrete-command-fd");
    args.push_back(std::to_string(discrete_command_fds[1]));
    args.push_back("--image-fd");
    args.push_back(std::to_string(image_fds[1]));
    args.push_back("--lidar-fd");
    args.push_back(std::to_string(lidar_fds[1]));

    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (std::string& arg : args) {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);

    execv(launch_config_.executable_path.c_str(), argv.data());
    std::perror("execv ausim_ros_bridge");
    _exit(127);
  }

  close(telemetry_fds[1]);
  close(command_fds[1]);
  close(discrete_command_fds[1]);
  close(image_fds[1]);
  close(lidar_fds[1]);

  running_.store(true);
  command_thread_ = std::thread(&RosBridgeProcessManager::CommandLoop, this);
  discrete_command_thread_ = std::thread(&RosBridgeProcessManager::DiscreteCommandLoop, this);
  telemetry_thread_ = std::thread(&RosBridgeProcessManager::TelemetryLoop, this);
  if (!camera_channel_names_.empty()) {
    camera_thread_ = std::thread(&RosBridgeProcessManager::CameraLoop, this);
  }
  // Start lidar loop if any lidar sensor is configured.
  for (const ausim::SensorConfig& s : config_.sensors) {
    if (s.type == "lidar" && s.enabled) {
      lidar_thread_ = std::thread(&RosBridgeProcessManager::LidarLoop, this);
      break;
    }
  }
  if (config_.dynamic_obstacle.enabled && config_.dynamic_obstacle.publish.enabled) {
    dynamic_obstacles_thread_ = std::thread(&RosBridgeProcessManager::DynamicObstaclesLoop, this);
  }

  std::this_thread::sleep_for(kStartupProbeDelay);
  EnsureChildStillRunning("startup");
}

void RosBridgeProcessManager::Stop() {
  const bool was_running = running_.exchange(false);

  ipc::ShutdownAndClose(&telemetry_send_fd_);
  ipc::ShutdownAndClose(&command_recv_fd_);
  ipc::ShutdownAndClose(&discrete_command_recv_fd_);
  ipc::ShutdownAndClose(&image_send_fd_);
  ipc::ShutdownAndClose(&lidar_send_fd_);

  if (telemetry_thread_.joinable()) {
    telemetry_thread_.join();
  }
  if (command_thread_.joinable()) {
    command_thread_.join();
  }
  if (discrete_command_thread_.joinable()) {
    discrete_command_thread_.join();
  }
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  if (lidar_thread_.joinable()) {
    lidar_thread_.join();
  }
  if (dynamic_obstacles_thread_.joinable()) {
    dynamic_obstacles_thread_.join();
  }

  if (child_pid_ > 0) {
    int status = 0;
    pid_t result = waitpid(child_pid_, &status, WNOHANG);
    if (result == 0) {
      kill(child_pid_, SIGTERM);
      for (int i = 0; i < kShutdownPollCount; ++i) {
        std::this_thread::sleep_for(kShutdownPollDelay);
        result = waitpid(child_pid_, &status, WNOHANG);
        if (result == child_pid_) {
          break;
        }
      }
      if (result == 0) {
        kill(child_pid_, SIGKILL);
        waitpid(child_pid_, &status, 0);
      }
    } else if (result < 0 && errno != ECHILD && was_running) {
      std::cerr << "ausim warning: waitpid failed while stopping ROS bridge: " << std::strerror(errno) << '\n';
    }
    child_pid_ = -1;
  }
}

void RosBridgeProcessManager::TelemetryLoop() {
  const auto period = std::chrono::duration<double>(1.0 / config_.ros2.publish_rate_hz);
  auto next_tick = std::chrono::steady_clock::now();

  while (running_.load()) {
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    const std::optional<TelemetrySnapshot> snapshot = ReadTelemetrySnapshot();
    if (snapshot.has_value()) {
      const ipc::TelemetryPacket packet = converts::ToTelemetryPacket(*snapshot);
      const std::vector<std::uint8_t> message = BuildTaggedMessage(ipc::BridgeMessageType::kTelemetry, &packet, sizeof(packet));
      std::lock_guard<std::mutex> lock(telemetry_send_mutex_);
      ipc::SendPacketBytes(telemetry_send_fd_, message.data(), message.size(), true);
    }
    std::this_thread::sleep_until(next_tick);
  }
}

void RosBridgeProcessManager::DynamicObstaclesLoop() {
  const auto period = std::chrono::duration<double>(1.0 / config_.dynamic_obstacle.publish.rate_hz);
  auto next_tick = std::chrono::steady_clock::now();
  double last_sent_sim_time = -1.0;

  while (running_.load()) {
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    const std::optional<DynamicObstaclesSnapshot> snapshot = ReadDynamicObstaclesSnapshot();
    if (snapshot.has_value() && std::abs(snapshot->sim_time - last_sent_sim_time) > 1e-9) {
      std::vector<std::uint8_t> payload;
      if (converts::ToDynObstaclePacket(*snapshot, payload)) {
        const std::vector<std::uint8_t> message = BuildTaggedMessage(ipc::BridgeMessageType::kDynamicObstacles, payload.data(), payload.size());
        std::lock_guard<std::mutex> lock(telemetry_send_mutex_);
        if (ipc::SendPacketBytes(telemetry_send_fd_, message.data(), message.size(), true)) {
          last_sent_sim_time = snapshot->sim_time;
        }
      }
    }
    std::this_thread::sleep_until(next_tick);
  }
}

void RosBridgeProcessManager::LidarLoop() {
  // Find the first enabled lidar sensor config for rate and name.
  double rate_hz = 10.0;
  std::string lidar_name;
  for (const ausim::SensorConfig& s : config_.sensors) {
    if (s.type == "lidar" && s.enabled) {
      rate_hz = s.rate_hz > 0.0 ? s.rate_hz : 10.0;
      lidar_name = s.name;
      break;
    }
  }
  if (lidar_name.empty()) {
    return;
  }

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  auto next_tick = std::chrono::steady_clock::now();

  while (running_.load()) {
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

    const std::optional<LidarSnapshot> snap = ReadLidarSnapshot(lidar_name);
    if (snap.has_value()) {
      const int n = static_cast<int>(snap->ranges.size());
      if (n > 0 && n <= ipc::LidarPacket::kMaxRays) {
        ipc::LidarPacket packet;
        packet.sim_time = snap->sim_time;
        packet.h_ray_num = snap->h_ray_num;
        packet.v_ray_num = snap->v_ray_num;
        packet.fov_h_deg = snap->fov_h_deg;
        packet.fov_v_deg = snap->fov_v_deg;
        packet.range_min_m = snap->range_min_m;
        packet.range_max_m = snap->range_max_m;
        for (int i = 0; i < n; ++i) {
          packet.data[i] = snap->ranges[i];
        }
        ipc::SendPacket(lidar_send_fd_, packet, true);
      }
    }

    std::this_thread::sleep_until(next_tick);
  }
}

void RosBridgeProcessManager::CommandLoop() {
  while (running_.load()) {
    ipc::VelocityCommandPacket packet;
    switch (ipc::ReceivePacket(command_recv_fd_, &packet)) {
      case ipc::PacketReceiveStatus::kPacket:
        WriteVelocityCommand(converts::ToVelocityCommand(packet, std::chrono::steady_clock::now()));
        break;
      case ipc::PacketReceiveStatus::kClosed:
        running_.store(false);
        return;
      case ipc::PacketReceiveStatus::kWouldBlock:
        continue;
      case ipc::PacketReceiveStatus::kError:
        if (running_.load()) {
          std::cerr << "ausim warning: command socket receive failed\n";
        }
        running_.store(false);
        return;
    }
  }
}

void RosBridgeProcessManager::DiscreteCommandLoop() {
  while (running_.load()) {
    ipc::DiscreteCommandPacket packet;
    switch (ipc::ReceivePacket(discrete_command_recv_fd_, &packet)) {
      case ipc::PacketReceiveStatus::kPacket:
        WriteDiscreteCommand(converts::ToDiscreteCommand(packet, std::chrono::steady_clock::now()));
        break;
      case ipc::PacketReceiveStatus::kClosed:
        running_.store(false);
        return;
      case ipc::PacketReceiveStatus::kWouldBlock:
        continue;
      case ipc::PacketReceiveStatus::kError:
        if (running_.load()) {
          std::cerr << "ausim warning: discrete command socket receive failed\n";
        }
        running_.store(false);
        return;
    }
  }
}

void RosBridgeProcessManager::CameraLoop() {
  const auto period = std::chrono::duration<double>(1.0 / config_.ros2.publish_rate_hz);
  auto next_tick = std::chrono::steady_clock::now();
  std::vector<std::uint32_t> last_sent_sequences(camera_channel_names_.size(), 0);

  while (running_.load()) {
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

    for (std::size_t i = 0; i < camera_channel_names_.size() && running_.load(); ++i) {
      const std::optional<CameraFrame> frame = ReadCameraFrame(camera_channel_names_[i]);
      if (!frame.has_value() || frame->sequence == last_sent_sequences[i] || frame->data.empty()) {
        continue;
      }

      const ipc::CameraImageMetadataPacket metadata = converts::ToCameraImageMetadataPacket(*frame, static_cast<std::uint32_t>(i));
      const bool sent =
          ipc::WriteFully(image_send_fd_, &metadata, sizeof(metadata)) && ipc::WriteFully(image_send_fd_, frame->data.data(), frame->data.size());
      if (!sent) {
        if (running_.load()) {
          std::cerr << "ausim warning: image socket send failed\n";
        }
        running_.store(false);
        return;
      }
      last_sent_sequences[i] = frame->sequence;
    }

    std::this_thread::sleep_until(next_tick);
  }
}

void RosBridgeProcessManager::EnsureChildStillRunning(const char* stage) const {
  if (child_pid_ <= 0) {
    throw std::runtime_error("ROS bridge process was not started.");
  }

  int status = 0;
  const pid_t result = waitpid(child_pid_, &status, WNOHANG);
  if (result == 0) {
    return;
  }
  if (result < 0) {
    throw SystemError("Failed to probe ROS bridge process during " + std::string(stage));
  }

  if (WIFEXITED(status)) {
    throw std::runtime_error("ROS bridge process exited during " + std::string(stage) + " with code " + std::to_string(WEXITSTATUS(status)) + ".");
  }
  if (WIFSIGNALED(status)) {
    throw std::runtime_error("ROS bridge process exited during " + std::string(stage) + " with signal " + std::to_string(WTERMSIG(status)) + ".");
  }

  throw std::runtime_error("ROS bridge process exited unexpectedly during " + std::string(stage) + ".");
}

}  // namespace ausim
