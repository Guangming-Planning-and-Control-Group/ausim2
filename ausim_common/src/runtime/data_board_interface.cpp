#include "runtime/data_board_interface.hpp"

#include <algorithm>
#include <chrono>
#include <string>

#include "runtime/data_board_channels.hpp"

namespace ausim {
namespace {

std::string CameraFrameChannel(const std::string& channel_name) { return std::string(runtime_board::kCameraFramePrefix) + channel_name; }

std::string LidarSnapshotChannel(const std::string& name) { return std::string(runtime_board::kLidarSnapshotPrefix) + name; }

}  // namespace

db::SecurityDataRef<VelocityCommand, db::Permission::ReadOnly> VelocityCommandReader() {
  static auto reader = db::DataBoard().Read<VelocityCommand>(runtime_board::kVelocityCommand);
  return reader;
}

db::SecurityDataRef<VelocityCommand, db::Permission::ReadWrite> VelocityCommandWriter() {
  static auto writer = db::DataBoard().Write<VelocityCommand>(runtime_board::kVelocityCommand);
  return writer;
}

db::SecurityDataRef<DiscreteCommand, db::Permission::ReadOnly> DiscreteCommandReader() {
  static auto reader = db::DataBoard().Read<DiscreteCommand>(runtime_board::kDiscreteCommand);
  return reader;
}

db::SecurityDataRef<DiscreteCommand, db::Permission::ReadWrite> DiscreteCommandWriter() {
  static auto writer = db::DataBoard().Write<DiscreteCommand>(runtime_board::kDiscreteCommand);
  return writer;
}

db::SecurityDataRef<TelemetrySnapshot, db::Permission::ReadOnly> TelemetrySnapshotReader() {
  static auto reader = db::DataBoard().Read<TelemetrySnapshot>(runtime_board::kTelemetrySnapshot);
  return reader;
}

db::SecurityDataRef<TelemetrySnapshot, db::Permission::ReadWrite> TelemetrySnapshotWriter() {
  static auto writer = db::DataBoard().Write<TelemetrySnapshot>(runtime_board::kTelemetrySnapshot);
  return writer;
}

db::SecurityDataRef<DynamicObstaclesSnapshot, db::Permission::ReadOnly> DynamicObstaclesSnapshotReader() {
  static auto reader = db::DataBoard().Read<DynamicObstaclesSnapshot>(runtime_board::kDynamicObstaclesSnapshot);
  return reader;
}

db::SecurityDataRef<DynamicObstaclesSnapshot, db::Permission::ReadWrite> DynamicObstaclesSnapshotWriter() {
  static auto writer = db::DataBoard().Write<DynamicObstaclesSnapshot>(runtime_board::kDynamicObstaclesSnapshot);
  return writer;
}

db::SecurityDataRef<CameraFrame, db::Permission::ReadOnly> CameraFrameReader(const std::string& channel_name) {
  return db::DataBoard().Read<CameraFrame>(CameraFrameChannel(channel_name));
}

db::SecurityDataRef<CameraFrame, db::Permission::ReadWrite> CameraFrameWriter(const std::string& channel_name) {
  return db::DataBoard().Write<CameraFrame>(CameraFrameChannel(channel_name));
}

std::optional<VelocityCommand> ReadVelocityCommand() { return VelocityCommandReader().ReadOptional(); }

std::optional<VelocityCommand> ReadFreshVelocityCommand(double timeout_seconds) {
  const std::optional<VelocityCommand> command = ReadVelocityCommand();
  if (!command.has_value()) {
    return std::nullopt;
  }

  const double clamped_timeout_seconds = std::max(0.0, timeout_seconds);
  const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - command->received_time);
  if (age.count() > clamped_timeout_seconds) {
    return std::nullopt;
  }

  return command;
}

void WriteVelocityCommand(const VelocityCommand& command) { VelocityCommandWriter() = command; }

void ClearVelocityCommand() { VelocityCommandWriter().Reset(); }

std::optional<DiscreteCommand> ReadDiscreteCommand() { return DiscreteCommandReader().ReadOptional(); }

std::optional<DiscreteCommand> ReadFreshDiscreteCommand(double timeout_seconds) {
  const std::optional<DiscreteCommand> command = ReadDiscreteCommand();
  if (!command.has_value()) {
    return std::nullopt;
  }

  const double clamped_timeout_seconds = std::max(0.0, timeout_seconds);
  const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - command->received_time);
  if (age.count() > clamped_timeout_seconds) {
    return std::nullopt;
  }

  return command;
}

void WriteDiscreteCommand(const DiscreteCommand& command) { DiscreteCommandWriter() = command; }

void ClearDiscreteCommand() { DiscreteCommandWriter().Reset(); }

std::optional<TelemetrySnapshot> ReadTelemetrySnapshot() { return TelemetrySnapshotReader().ReadOptional(); }

void WriteTelemetrySnapshot(const TelemetrySnapshot& snapshot) { TelemetrySnapshotWriter() = snapshot; }

std::optional<DynamicObstaclesSnapshot> ReadDynamicObstaclesSnapshot() { return DynamicObstaclesSnapshotReader().ReadOptional(); }

void WriteDynamicObstaclesSnapshot(const DynamicObstaclesSnapshot& snapshot) { DynamicObstaclesSnapshotWriter() = snapshot; }

std::optional<CameraFrame> ReadCameraFrame(const std::string& channel_name) { return CameraFrameReader(channel_name).ReadOptional(); }

void WriteCameraFrame(const std::string& channel_name, const CameraFrame& frame) { CameraFrameWriter(channel_name) = frame; }

std::optional<LidarSnapshot> ReadLidarSnapshot(const std::string& name) {
  return db::DataBoard().Read<LidarSnapshot>(LidarSnapshotChannel(name)).ReadOptional();
}

void WriteLidarSnapshot(const std::string& name, const LidarSnapshot& snap) {
  db::DataBoard().Write<LidarSnapshot>(LidarSnapshotChannel(name)) = snap;
}

}  // namespace ausim
