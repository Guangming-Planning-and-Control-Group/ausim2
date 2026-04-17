#pragma once

#include <chrono>
#include <string>

#include "data/clock.hpp"
#include "data/cmd_vel.hpp"
#include "data/image.hpp"
#include "data/imu.hpp"
#include "data/odom.hpp"
#include "data/transform.hpp"
#include "ipc/bridge_packets.hpp"
#include "runtime/runtime_types.hpp"

namespace ausim::converts {

ipc::VelocityCommandPacket ToVelocityCommandPacket(const data::CmdVelData& message);
VelocityCommand ToVelocityCommand(const ipc::VelocityCommandPacket& packet, std::chrono::steady_clock::time_point received_time);
ipc::DiscreteCommandPacket ToDiscreteCommandPacket(const DiscreteCommand& command);
DiscreteCommand ToDiscreteCommand(const ipc::DiscreteCommandPacket& packet, std::chrono::steady_clock::time_point received_time);

ipc::TelemetryPacket ToTelemetryPacket(const TelemetrySnapshot& snapshot);
ipc::CameraImageMetadataPacket ToCameraImageMetadataPacket(const CameraFrame& frame, std::uint32_t sensor_index);
CameraFrame ToCameraFrame(const ipc::CameraImageMetadataPacket& packet, std::vector<std::uint8_t> pixels);

data::OdomData ToOdomData(const ipc::TelemetryPacket& packet, const std::string& frame_id, const std::string& child_frame_id);
data::ImuData ToImuData(const ipc::TelemetryPacket& packet, const std::string& frame_id);
data::TransformData ToTransformData(const ipc::TelemetryPacket& packet, const std::string& frame_id, const std::string& child_frame_id);
data::ClockData ToClockData(const ipc::TelemetryPacket& packet);
data::ImageData ToImageData(const CameraFrame& frame, const std::string& frame_id);

}  // namespace ausim::converts
