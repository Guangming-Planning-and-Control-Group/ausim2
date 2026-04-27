#include "converts/ipc/bridge_packets.hpp"

#include <algorithm>
#include <cstring>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ausim::converts {
namespace {

data::Vector3 BuildVector3(const std::array<double, 3>& value) { return data::Vector3{value[0], value[1], value[2]}; }

data::Quaternion BuildQuaternion(const std::array<double, 4>& value) { return data::Quaternion{value[0], value[1], value[2], value[3]}; }

data::Header BuildHeader(double stamp_seconds, const std::string& frame_id) {
  data::Header header;
  header.stamp_seconds = stamp_seconds;
  header.frame_id = frame_id;
  return header;
}

Eigen::Vector3d ToEigenVector3(const std::array<double, 3>& value) { return Eigen::Vector3d(value[0], value[1], value[2]); }

Eigen::Quaterniond ToEigenQuaternion(const std::array<double, 4>& value) { return Eigen::Quaterniond(value[0], value[1], value[2], value[3]); }

std::array<char, ipc::kRobotModeNameCapacity> ToFixedString(const std::string& value) {
  std::array<char, ipc::kRobotModeNameCapacity> buffer = {};
  const std::size_t copy_length = std::min<std::size_t>(value.size(), buffer.size() - 1);
  std::copy_n(value.data(), copy_length, buffer.data());
  return buffer;
}

std::array<char, ipc::kDiscreteEventNameCapacity> ToDiscreteEventName(const std::string& value) {
  std::array<char, ipc::kDiscreteEventNameCapacity> buffer = {};
  const std::size_t copy_length = std::min<std::size_t>(value.size(), buffer.size() - 1);
  std::copy_n(value.data(), copy_length, buffer.data());
  return buffer;
}

template <std::size_t Capacity>
std::array<char, Capacity> ToFixedCString(const std::string& value) {
  std::array<char, Capacity> buffer = {};
  const std::size_t copy_length = std::min<std::size_t>(value.size(), buffer.size() - 1);
  std::copy_n(value.data(), copy_length, buffer.data());
  return buffer;
}

template <std::size_t Capacity>
std::string FromFixedCString(const std::array<char, Capacity>& buffer) {
  const auto nul = std::find(buffer.begin(), buffer.end(), '\0');
  return std::string(buffer.data(), static_cast<std::size_t>(std::distance(buffer.begin(), nul)));
}

std::string FromDiscreteEventName(const std::array<char, ipc::kDiscreteEventNameCapacity>& buffer) {
  const auto nul = std::find(buffer.begin(), buffer.end(), '\0');
  return std::string(buffer.data(), static_cast<std::size_t>(std::distance(buffer.begin(), nul)));
}

}  // namespace

ipc::VelocityCommandPacket ToVelocityCommandPacket(const data::CmdVelData& message) {
  ipc::VelocityCommandPacket packet;
  packet.linear = {
      message.twist.linear.x,
      message.twist.linear.y,
      message.twist.linear.z,
  };
  packet.angular = {
      message.twist.angular.x,
      message.twist.angular.y,
      message.twist.angular.z,
  };
  return packet;
}

VelocityCommand ToVelocityCommand(const ipc::VelocityCommandPacket& packet, std::chrono::steady_clock::time_point received_time) {
  VelocityCommand command;
  command.linear = ToEigenVector3(packet.linear);
  command.angular = ToEigenVector3(packet.angular);
  command.received_time = received_time;
  return command;
}

ipc::DiscreteCommandPacket ToDiscreteCommandPacket(const DiscreteCommand& command) {
  ipc::DiscreteCommandPacket packet;
  packet.sequence = command.sequence;
  packet.event = ToDiscreteEventName(command.event_name);
  return packet;
}

DiscreteCommand ToDiscreteCommand(const ipc::DiscreteCommandPacket& packet, std::chrono::steady_clock::time_point received_time) {
  DiscreteCommand command;
  command.event_name = FromDiscreteEventName(packet.event);
  command.kind = ClassifyDiscreteEvent(command.event_name);
  command.sequence = packet.sequence;
  command.received_time = received_time;
  return command;
}

ipc::TelemetryPacket ToTelemetryPacket(const TelemetrySnapshot& snapshot) {
  ipc::TelemetryPacket packet;
  packet.sim_time = snapshot.sim_time;
  packet.position = {
      snapshot.state.position.x(),
      snapshot.state.position.y(),
      snapshot.state.position.z(),
  };
  packet.linear_velocity = {
      snapshot.state.velocity.x(),
      snapshot.state.velocity.y(),
      snapshot.state.velocity.z(),
  };
  packet.orientation = {
      snapshot.state.quaternion.w(),
      snapshot.state.quaternion.x(),
      snapshot.state.quaternion.y(),
      snapshot.state.quaternion.z(),
  };
  packet.angular_velocity = {
      snapshot.state.omega.x(),
      snapshot.state.omega.y(),
      snapshot.state.omega.z(),
  };
  packet.imu_orientation = {
      snapshot.imu.orientation.w(),
      snapshot.imu.orientation.x(),
      snapshot.imu.orientation.y(),
      snapshot.imu.orientation.z(),
  };
  packet.imu_angular_velocity = {
      snapshot.imu.angular_velocity.x(),
      snapshot.imu.angular_velocity.y(),
      snapshot.imu.angular_velocity.z(),
  };
  packet.imu_linear_acceleration = {
      snapshot.imu.linear_acceleration.x(),
      snapshot.imu.linear_acceleration.y(),
      snapshot.imu.linear_acceleration.z(),
  };
  packet.imu_has_linear_acceleration = snapshot.imu.has_linear_acceleration ? 1 : 0;
  packet.robot_top_level_state = static_cast<std::uint8_t>(snapshot.robot_mode.top_state);
  packet.robot_mode_accepts_motion = snapshot.robot_mode.accepts_motion ? 1 : 0;
  packet.robot_mode_sub_state = ToFixedString(snapshot.robot_mode.sub_state);
  packet.last_discrete_command_status = static_cast<std::uint8_t>(snapshot.last_discrete_command_status);
  packet.last_discrete_command_sequence = snapshot.last_discrete_command_sequence;
  return packet;
}

bool ToDynObstaclePacket(const DynamicObstaclesSnapshot& snapshot, std::vector<std::uint8_t>& out) {
  if (snapshot.entries.size() > ipc::kMaxDynObstacleEntries) {
    return false;
  }

  const std::size_t total_size = sizeof(ipc::DynObstaclePacketHeader) + snapshot.entries.size() * sizeof(ipc::DynObstaclePacketEntry);
  out.resize(total_size);

  ipc::DynObstaclePacketHeader header;
  header.magic = 0x444F4253u;  // "DOBS"
  header.entry_count = static_cast<std::uint32_t>(snapshot.entries.size());
  header.sim_time = snapshot.sim_time;
  header.frame_id = ToFixedCString<ipc::kDynObstacleFrameIdCapacity>(snapshot.frame_id);
  std::memcpy(out.data(), &header, sizeof(header));

  auto* entries = reinterpret_cast<ipc::DynObstaclePacketEntry*>(out.data() + sizeof(header));
  for (std::size_t index = 0; index < snapshot.entries.size(); ++index) {
    const DynamicObstacleEntry& in = snapshot.entries[index];
    ipc::DynObstaclePacketEntry& entry = entries[index];
    entry.name = ToFixedCString<ipc::kDynObstacleNameCapacity>(in.name);
    std::copy_n(in.pos, 3, entry.pos.begin());
    std::copy_n(in.quat, 4, entry.quat.begin());
    std::copy_n(in.size, 3, entry.size.begin());
  }

  return true;
}

bool FromDynObstaclePacketBytes(const std::uint8_t* data, std::size_t len, DynamicObstaclesSnapshot& out) {
  out = DynamicObstaclesSnapshot{};
  if (data == nullptr || len < sizeof(ipc::DynObstaclePacketHeader)) {
    return false;
  }

  ipc::DynObstaclePacketHeader header;
  std::memcpy(&header, data, sizeof(header));
  if (header.magic != 0x444F4253u) {
    return false;
  }
  if (header.entry_count > ipc::kMaxDynObstacleEntries) {
    return false;
  }

  const std::size_t expected_size =
      sizeof(ipc::DynObstaclePacketHeader) + static_cast<std::size_t>(header.entry_count) * sizeof(ipc::DynObstaclePacketEntry);
  if (len != expected_size) {
    return false;
  }

  out.sim_time = header.sim_time;
  out.frame_id = FromFixedCString(header.frame_id);
  out.entries.resize(header.entry_count);

  const auto* entries = reinterpret_cast<const ipc::DynObstaclePacketEntry*>(data + sizeof(header));
  for (std::size_t index = 0; index < out.entries.size(); ++index) {
    const ipc::DynObstaclePacketEntry& in = entries[index];
    DynamicObstacleEntry& entry = out.entries[index];
    entry.name = FromFixedCString(in.name);
    std::copy(in.pos.begin(), in.pos.end(), entry.pos);
    std::copy(in.quat.begin(), in.quat.end(), entry.quat);
    std::copy(in.size.begin(), in.size.end(), entry.size);
  }

  return true;
}

ipc::CameraImageMetadataPacket ToCameraImageMetadataPacket(const CameraFrame& frame, std::uint32_t sensor_index) {
  ipc::CameraImageMetadataPacket packet;
  packet.sim_time = frame.sim_time;
  packet.sensor_index = sensor_index;
  packet.sequence = frame.sequence;
  packet.width = frame.width;
  packet.height = frame.height;
  packet.step = frame.step;
  packet.pixel_format = static_cast<std::uint32_t>(frame.format);
  packet.data_size = static_cast<std::uint32_t>(frame.data.size());
  return packet;
}

CameraFrame ToCameraFrame(const ipc::CameraImageMetadataPacket& packet, std::vector<std::uint8_t> pixels) {
  CameraFrame frame;
  frame.sim_time = packet.sim_time;
  frame.sequence = packet.sequence;
  frame.width = packet.width;
  frame.height = packet.height;
  frame.step = packet.step;
  frame.format = static_cast<CameraFrameFormat>(packet.pixel_format);
  frame.data = std::move(pixels);
  return frame;
}

data::OdomData ToOdomData(const ipc::TelemetryPacket& packet, const std::string& frame_id, const std::string& child_frame_id) {
  data::OdomData message;
  message.header = BuildHeader(packet.sim_time, frame_id);
  message.child_frame_id = child_frame_id;
  message.pose.position = BuildVector3(packet.position);
  message.pose.orientation = BuildQuaternion(packet.orientation);
  message.twist.linear = BuildVector3(packet.linear_velocity);
  message.twist.angular = BuildVector3(packet.angular_velocity);
  return message;
}

data::ImuData ToImuData(const ipc::TelemetryPacket& packet, const std::string& frame_id) {
  data::ImuData message;
  message.header = BuildHeader(packet.sim_time, frame_id);
  message.orientation = BuildQuaternion(packet.imu_orientation);
  message.angular_velocity = BuildVector3(packet.imu_angular_velocity);
  message.linear_acceleration = BuildVector3(packet.imu_linear_acceleration);
  message.has_linear_acceleration = packet.imu_has_linear_acceleration != 0;
  return message;
}

data::TransformData ToTransformData(const ipc::TelemetryPacket& packet, const std::string& frame_id, const std::string& child_frame_id) {
  data::TransformData message;
  message.header = BuildHeader(packet.sim_time, frame_id);
  message.child_frame_id = child_frame_id;
  message.translation = BuildVector3(packet.position);
  message.rotation = BuildQuaternion(packet.orientation);
  return message;
}

data::ClockData ToClockData(const ipc::TelemetryPacket& packet) {
  data::ClockData message;
  message.stamp_seconds = packet.sim_time;
  return message;
}

data::ImageData ToImageData(const CameraFrame& frame, const std::string& frame_id) {
  data::ImageData message;
  message.header = BuildHeader(frame.sim_time, frame_id);
  message.width = frame.width;
  message.height = frame.height;
  if (frame.format == CameraFrameFormat::kDepth32F) {
    message.encoding = "32FC1";
  } else {
    message.encoding = "rgb8";
  }
  message.step = frame.step;
  message.data = frame.data;
  return message;
}

}  // namespace ausim::converts
