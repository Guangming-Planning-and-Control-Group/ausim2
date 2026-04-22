# ROS Message Architecture

`ausim_msg` is the semantic message layer for ausim2. It does not replace
standard ROS interfaces such as `geometry_msgs/Twist`, `nav_msgs/Odometry`,
`sensor_msgs/Image`, `sensor_msgs/Imu`, or `sensor_msgs/PointCloud2`.

## File Form

- `third_party/ausim_msg/msg/*.msg`
  - shared semantic interfaces available through ROS overlay `source`
  - includes vision-style example messages plus:
    - `RobotMode.msg`
    - `SimulationEvent.msg`
    - `SimulationEventAck.msg`
    - `DeviceCapability.msg`
    - `DeviceStatus.msg`
- `ausim_common/src/converts/ausim_msg/*.cpp`
  - internal snapshot / IPC packet to `ausim_msg` conversion
- `ausim_common/src/ros/publisher/semantic/*.cpp`
  - structured semantic publishers
- `ausim_common/src/ros/publisher/data/*.cpp`
  - standard ROS data publishers

## Data Flow

- control input:
  - `Twist` / `Trigger`
  - `-> ros subscriber`
  - `-> internal command`
  - `-> DataBoard / runtime`
  - `-> mjData->ctrl`
- telemetry output:
  - `mjModel/mjData`
  - `-> sim reader`
  - `-> TelemetrySnapshot`
  - `-> ipc::TelemetryPacket`
  - `-> ros bridge`
  - `-> standard ROS msgs`
  - `-> ausim_msg semantic msgs`

## Current Data-Flow Diagram

```mermaid
flowchart LR
    A[mjModel / mjData] --> B[sim reader / extractor]
    B --> C[TelemetrySnapshot / CameraFrame / LidarSnapshot]
    C --> D[ipc::TelemetryPacket / image metadata + buffer / lidar packet]
    D --> E[ROS bridge]
    E --> F[Standard ROS messages]
    E --> G[ausim_msg semantic messages]

    F --> F1[Odometry]
    F --> F2[Imu]
    F --> F3[Image]
    F --> F4[PointCloud2]
    F --> F5[Clock]
    F --> F6[TF]

    G --> G1[RobotMode]
    G --> G2[SimulationEventAck]
    G --> G3[DeviceStatus]
    G --> G4[Vision-style detections]
```

## ROS Topic <-> Simulator Bidirectional Chain

```mermaid
flowchart LR
    subgraph Input[ROS to Simulator]
        I1[geometry_msgs/Twist] --> I2[ROS subscriber]
        I1b[std_srvs/Trigger] --> I2
        I1c[future ausim_msg command] --> I2
        I2 --> I3[internal command]
        I3 --> I4[DataBoard / runtime queue]
        I4 --> I5[controller / mode machine]
        I5 --> I6[mjData ctrl]
    end

    subgraph Output[Simulator to ROS]
        O1[mjData / mjModel] --> O2[reader / extractor]
        O2 --> O3[neutral snapshot]
        O3 --> O4[IPC packet]
        O4 --> O5[ROS bridge]
        O5 --> O6[standard ROS topic]
        O5 --> O7[ausim_msg topic]
    end
```

## Current Structured Route

The first live semantic route is:

`mjData -> TelemetrySnapshot -> ipc::TelemetryPacket -> RobotModePublisher -> ausim_msg/msg/RobotMode`

The legacy compatibility route remains in parallel:

`mjData -> TelemetrySnapshot -> ipc::TelemetryPacket -> std_msgs/String(JSON)`
