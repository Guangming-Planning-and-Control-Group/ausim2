#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace ausim::ipc {

struct VelocityCommandPacket {
  std::array<double, 3> linear = {0.0, 0.0, 0.0};
  std::array<double, 3> angular = {0.0, 0.0, 0.0};
};

struct DiscreteCommandPacket {
  std::uint8_t kind = 0;
  std::uint64_t sequence = 0;
};

inline constexpr std::size_t kRobotModeNameCapacity = 32;

struct TelemetryPacket {
  double sim_time = 0.0;
  std::array<double, 3> position = {0.0, 0.0, 0.0};
  std::array<double, 3> linear_velocity = {0.0, 0.0, 0.0};
  std::array<double, 4> orientation = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0};
  std::array<double, 4> imu_orientation = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> imu_angular_velocity = {0.0, 0.0, 0.0};
  std::array<double, 3> imu_linear_acceleration = {0.0, 0.0, 0.0};
  std::uint8_t imu_has_linear_acceleration = 0;
  std::uint8_t robot_top_level_state = 0;
  std::uint8_t robot_mode_accepts_motion = 0;
  std::array<char, kRobotModeNameCapacity> robot_mode_sub_state = {};
  std::uint8_t last_discrete_command_status = 0;
  std::uint64_t last_discrete_command_sequence = 0;
};

struct CameraImageMetadataPacket {
  double sim_time = 0.0;
  std::uint32_t sensor_index = 0;
  std::uint32_t sequence = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  std::uint32_t pixel_format = 0;
  std::uint32_t data_size = 0;
};

static_assert(std::is_trivially_copyable_v<VelocityCommandPacket>);
static_assert(std::is_trivially_copyable_v<DiscreteCommandPacket>);
static_assert(std::is_trivially_copyable_v<TelemetryPacket>);
static_assert(std::is_trivially_copyable_v<CameraImageMetadataPacket>);

}  // namespace ausim::ipc
