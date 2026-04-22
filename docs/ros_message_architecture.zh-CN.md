# ROS 消息架构

`ausim_msg` 是 ausim2 的语义消息层。它不会替代标准 ROS 接口，例如 `geometry_msgs/Twist`、`nav_msgs/Odometry`、`sensor_msgs/Image`、`sensor_msgs/Imu` 或 `sensor_msgs/PointCloud2`。

## 文件形式

- `third_party/ausim_msg/msg/*.msg`
  - 通过 ROS overlay `source` 可用的共享语义接口
  - 包含 vision 风格示例消息，以及：
    - `RobotMode.msg`
    - `SimulationEvent.msg`
    - `SimulationEventAck.msg`
    - `DeviceCapability.msg`
    - `DeviceStatus.msg`
- `ausim_common/src/converts/ausim_msg/*.cpp`
  - 内部快照 / IPC 包到 `ausim_msg` 的转换
- `ausim_common/src/ros/publisher/semantic/*.cpp`
  - 结构化语义发布器
- `ausim_common/src/ros/publisher/data/*.cpp`
  - 标准 ROS 数据发布器

## 数据流

- 控制输入：
  - `Twist` / `Trigger`
  - `-> ros subscriber`
  - `-> internal command`
  - `-> DataBoard / runtime`
  - `-> mjData->ctrl`
- 遥测输出：
  - `mjModel/mjData`
  - `-> sim reader`
  - `-> TelemetrySnapshot`
  - `-> ipc::TelemetryPacket`
  - `-> ros bridge`
  - `-> standard ROS msgs`
  - `-> ausim_msg semantic msgs`

## 当前数据流图

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

## ROS Topic <-> Simulator 双向链路

```mermaid
flowchart LR
    subgraph Input[ROS 到模拟器]
        I1[geometry_msgs/Twist] --> I2[ROS subscriber]
        I1b[std_srvs/Trigger] --> I2
        I1c[future ausim_msg command] --> I2
        I2 --> I3[internal command]
        I3 --> I4[DataBoard / runtime queue]
        I4 --> I5[controller / mode machine]
        I5 --> I6[mjData ctrl]
    end

    subgraph Output[模拟器到 ROS]
        O1[mjData / mjModel] --> O2[reader / extractor]
        O2 --> O3[neutral snapshot]
        O3 --> O4[IPC packet]
        O4 --> O5[ROS bridge]
        O5 --> O6[standard ROS topic]
        O5 --> O7[ausim_msg topic]
    end
```

## 当前结构化路线

第一条在线语义链路是：

`mjData -> TelemetrySnapshot -> ipc::TelemetryPacket -> RobotModePublisher -> ausim_msg/msg/RobotMode`

旧版兼容链路仍并行保留：

`mjData -> TelemetrySnapshot -> ipc::TelemetryPacket -> std_msgs/String(JSON)`
