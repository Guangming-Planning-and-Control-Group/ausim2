#include "converts/ipc/bridge_packets.hpp"

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
  return packet;
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
