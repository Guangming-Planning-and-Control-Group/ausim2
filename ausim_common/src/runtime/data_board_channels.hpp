#pragma once

namespace ausim::runtime_board {

inline constexpr char kVelocityCommand[] = "runtime.velocity_command";
inline constexpr char kDiscreteCommand[] = "runtime.discrete_command";
inline constexpr char kTelemetrySnapshot[] = "runtime.telemetry_snapshot";
inline constexpr char kDynamicObstaclesSnapshot[] = "runtime.dynamic_obstacles_snapshot";
inline constexpr char kCameraFramePrefix[] = "runtime.camera_frame.";
inline constexpr char kLidarSnapshotPrefix[] = "runtime.lidar_snapshot.";

}  // namespace ausim::runtime_board
