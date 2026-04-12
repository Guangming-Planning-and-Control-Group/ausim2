#include "runtime/data_board_interface.hpp"

#include <algorithm>
#include <chrono>

#include "runtime/data_board_channels.hpp"

namespace ausim {
namespace {

std::string CameraFrameChannel(const std::string& channel_name) {
  return std::string(runtime_board::kCameraFramePrefix) + channel_name;
}

}  // namespace

db::SecurityDataRef<VelocityCommand, db::Permission::ReadOnly> VelocityCommandReader() {
  static auto reader = db::DataBoard().Read<VelocityCommand>(runtime_board::kVelocityCommand);
  return reader;
}

db::SecurityDataRef<VelocityCommand, db::Permission::ReadWrite> VelocityCommandWriter() {
  static auto writer = db::DataBoard().Write<VelocityCommand>(runtime_board::kVelocityCommand);
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

db::SecurityDataRef<CameraFrame, db::Permission::ReadOnly> CameraFrameReader(
    const std::string& channel_name) {
  return db::DataBoard().Read<CameraFrame>(CameraFrameChannel(channel_name));
}

db::SecurityDataRef<CameraFrame, db::Permission::ReadWrite> CameraFrameWriter(
    const std::string& channel_name) {
  return db::DataBoard().Write<CameraFrame>(CameraFrameChannel(channel_name));
}

std::optional<VelocityCommand> ReadVelocityCommand() {
  return VelocityCommandReader().ReadOptional();
}

std::optional<VelocityCommand> ReadFreshVelocityCommand(double timeout_seconds) {
  const std::optional<VelocityCommand> command = ReadVelocityCommand();
  if (!command.has_value()) {
    return std::nullopt;
  }

  const double clamped_timeout_seconds = std::max(0.0, timeout_seconds);
  const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - command->received_time);
  if (age.count() > clamped_timeout_seconds) {
    return std::nullopt;
  }

  return command;
}

void WriteVelocityCommand(const VelocityCommand& command) {
  VelocityCommandWriter() = command;
}

std::optional<TelemetrySnapshot> ReadTelemetrySnapshot() {
  return TelemetrySnapshotReader().ReadOptional();
}

void WriteTelemetrySnapshot(const TelemetrySnapshot& snapshot) {
  TelemetrySnapshotWriter() = snapshot;
}

std::optional<CameraFrame> ReadCameraFrame(const std::string& channel_name) {
  return CameraFrameReader(channel_name).ReadOptional();
}

void WriteCameraFrame(const std::string& channel_name, const CameraFrame& frame) {
  CameraFrameWriter(channel_name) = frame;
}

}  // namespace ausim
