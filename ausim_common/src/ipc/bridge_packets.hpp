#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace ausim::ipc {

struct VelocityCommandPacket {
  std::array<double, 3> linear = {0.0, 0.0, 0.0};
  std::array<double, 3> angular = {0.0, 0.0, 0.0};
};

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
static_assert(std::is_trivially_copyable_v<TelemetryPacket>);
static_assert(std::is_trivially_copyable_v<CameraImageMetadataPacket>);

}  // namespace ausim::ipc
