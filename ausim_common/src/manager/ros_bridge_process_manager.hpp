#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "config/quadrotor_config.hpp"

namespace ausim {

struct RosBridgeLaunchConfig {
  std::filesystem::path executable_path;
  std::vector<std::string> config_arguments;
};

class RosBridgeProcessManager {
 public:
  RosBridgeProcessManager(const QuadrotorConfig& config, RosBridgeLaunchConfig launch_config);
  ~RosBridgeProcessManager();

  void Start();
  void Stop();

 private:
  void TelemetryLoop();
  void CommandLoop();
  void DiscreteCommandLoop();
  void CameraLoop();
  void LidarLoop();
  void DynamicObstaclesLoop();
  void EnsureChildStillRunning(const char* stage) const;

  QuadrotorConfig config_;
  RosBridgeLaunchConfig launch_config_;
  int telemetry_send_fd_ = -1;
  int command_recv_fd_ = -1;
  int discrete_command_recv_fd_ = -1;
  int image_send_fd_ = -1;
  int lidar_send_fd_ = -1;
  int child_pid_ = -1;
  std::atomic_bool running_ = false;
  std::thread telemetry_thread_;
  std::thread command_thread_;
  std::thread discrete_command_thread_;
  std::thread camera_thread_;
  std::thread lidar_thread_;
  std::thread dynamic_obstacles_thread_;
  std::vector<std::string> camera_channel_names_;
  std::mutex telemetry_send_mutex_;
};

}  // namespace ausim

namespace quadrotor {

using ::ausim::RosBridgeLaunchConfig;
using ::ausim::RosBridgeProcessManager;

}  // namespace quadrotor
