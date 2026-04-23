#pragma once

#include <optional>
#include <string>

#include "common/db/data_board.hpp"
#include "runtime/dynamic_obstacles_snapshot.hpp"
#include "runtime/lidar_snapshot.hpp"
#include "runtime/runtime_types.hpp"

namespace ausim {

db::SecurityDataRef<VelocityCommand, db::Permission::ReadOnly> VelocityCommandReader();
db::SecurityDataRef<VelocityCommand, db::Permission::ReadWrite> VelocityCommandWriter();
db::SecurityDataRef<DiscreteCommand, db::Permission::ReadOnly> DiscreteCommandReader();
db::SecurityDataRef<DiscreteCommand, db::Permission::ReadWrite> DiscreteCommandWriter();
db::SecurityDataRef<TelemetrySnapshot, db::Permission::ReadOnly> TelemetrySnapshotReader();
db::SecurityDataRef<TelemetrySnapshot, db::Permission::ReadWrite> TelemetrySnapshotWriter();
db::SecurityDataRef<DynamicObstaclesSnapshot, db::Permission::ReadOnly> DynamicObstaclesSnapshotReader();
db::SecurityDataRef<DynamicObstaclesSnapshot, db::Permission::ReadWrite> DynamicObstaclesSnapshotWriter();
db::SecurityDataRef<CameraFrame, db::Permission::ReadOnly> CameraFrameReader(const std::string& channel_name);
db::SecurityDataRef<CameraFrame, db::Permission::ReadWrite> CameraFrameWriter(const std::string& channel_name);

std::optional<VelocityCommand> ReadVelocityCommand();
std::optional<VelocityCommand> ReadFreshVelocityCommand(double timeout_seconds);
void WriteVelocityCommand(const VelocityCommand& command);
void ClearVelocityCommand();

std::optional<DiscreteCommand> ReadDiscreteCommand();
std::optional<DiscreteCommand> ReadFreshDiscreteCommand(double timeout_seconds);
void WriteDiscreteCommand(const DiscreteCommand& command);
void ClearDiscreteCommand();

std::optional<TelemetrySnapshot> ReadTelemetrySnapshot();
void WriteTelemetrySnapshot(const TelemetrySnapshot& snapshot);
std::optional<DynamicObstaclesSnapshot> ReadDynamicObstaclesSnapshot();
void WriteDynamicObstaclesSnapshot(const DynamicObstaclesSnapshot& snapshot);
std::optional<CameraFrame> ReadCameraFrame(const std::string& channel_name);
void WriteCameraFrame(const std::string& channel_name, const CameraFrame& frame);

std::optional<LidarSnapshot> ReadLidarSnapshot(const std::string& name);
void WriteLidarSnapshot(const std::string& name, const LidarSnapshot& snap);

}  // namespace ausim

namespace quadrotor {

using ::ausim::CameraFrameReader;
using ::ausim::CameraFrameWriter;
using ::ausim::ClearDiscreteCommand;
using ::ausim::ClearVelocityCommand;
using ::ausim::DiscreteCommandReader;
using ::ausim::DiscreteCommandWriter;
using ::ausim::ReadCameraFrame;
using ::ausim::ReadDiscreteCommand;
using ::ausim::ReadFreshDiscreteCommand;
using ::ausim::ReadFreshVelocityCommand;
using ::ausim::ReadTelemetrySnapshot;
using ::ausim::ReadVelocityCommand;
using ::ausim::TelemetrySnapshotReader;
using ::ausim::TelemetrySnapshotWriter;
using ::ausim::VelocityCommandReader;
using ::ausim::VelocityCommandWriter;
using ::ausim::WriteCameraFrame;
using ::ausim::WriteDiscreteCommand;
using ::ausim::WriteTelemetrySnapshot;
using ::ausim::WriteVelocityCommand;

}  // namespace quadrotor
