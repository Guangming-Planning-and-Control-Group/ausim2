#pragma once

#include <optional>

#include "common/db/data_board.hpp"
#include "runtime/runtime_types.hpp"

namespace ausim {

db::SecurityDataRef<VelocityCommand, db::Permission::ReadOnly> VelocityCommandReader();
db::SecurityDataRef<VelocityCommand, db::Permission::ReadWrite> VelocityCommandWriter();
db::SecurityDataRef<TelemetrySnapshot, db::Permission::ReadOnly> TelemetrySnapshotReader();
db::SecurityDataRef<TelemetrySnapshot, db::Permission::ReadWrite> TelemetrySnapshotWriter();
db::SecurityDataRef<CameraFrame, db::Permission::ReadOnly> CameraFrameReader(const std::string& channel_name);
db::SecurityDataRef<CameraFrame, db::Permission::ReadWrite> CameraFrameWriter(const std::string& channel_name);

std::optional<VelocityCommand> ReadVelocityCommand();
std::optional<VelocityCommand> ReadFreshVelocityCommand(double timeout_seconds);
void WriteVelocityCommand(const VelocityCommand& command);

std::optional<TelemetrySnapshot> ReadTelemetrySnapshot();
void WriteTelemetrySnapshot(const TelemetrySnapshot& snapshot);
std::optional<CameraFrame> ReadCameraFrame(const std::string& channel_name);
void WriteCameraFrame(const std::string& channel_name, const CameraFrame& frame);

}  // namespace ausim

namespace quadrotor {

using ::ausim::CameraFrameReader;
using ::ausim::CameraFrameWriter;
using ::ausim::ReadCameraFrame;
using ::ausim::ReadFreshVelocityCommand;
using ::ausim::ReadTelemetrySnapshot;
using ::ausim::ReadVelocityCommand;
using ::ausim::TelemetrySnapshotReader;
using ::ausim::TelemetrySnapshotWriter;
using ::ausim::VelocityCommandReader;
using ::ausim::VelocityCommandWriter;
using ::ausim::WriteCameraFrame;
using ::ausim::WriteTelemetrySnapshot;
using ::ausim::WriteVelocityCommand;

}  // namespace quadrotor
