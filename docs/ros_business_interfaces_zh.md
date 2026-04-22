# ROS 业务接口对照表（中文）

本文整理 `scout1` 当前配置下由 `ausim` 仿真桥对外暴露的业务 ROS 接口，只包含业务 `topic` 与 `service`。

不在本文范围内：
- ROS 2 默认系统 topic，例如 `parameter_events`、`rosout`
- 调试或临时测试接口

当前接口以 [ground_vehicle/cfg/robot/scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:38) 和
[ground_vehicle/cfg/sim_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/sim_config.yaml:17) 为准。

## 命名规则

- 配置中以 `/` 开头的名称是全局接口，不会挂到车辆命名空间下。
  - 例如：`/clock`、`/joy/cmd_vel`、`/joy/action3`
- 配置中不以 `/` 开头的名称会自动拼到车辆 namespace `/scout1` 下。
  - 例如：`cmd_vel -> /scout1/cmd_vel`
  - 例如：`teleop/mode_structured -> /scout1/teleop/mode_structured`
- `tf` 与 `tf_static` 由 TF broadcaster 走 ROS 标准全局总线，不挂载到 `/scout1` 前缀下。

## Topic 对照表

| Topic | 消息类型 | 方向 | 当前实例 | 作用 | 代码/配置来源 |
| --- | --- | --- | --- | --- | --- |
| `clock_topic` | `rosgraph_msgs/Clock` | 仿真桥发布 | `/clock` | 发布仿真时钟，供启用 `use_sim_time` 的 ROS 节点同步仿真时间。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:43) [sim_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/sim_config.yaml:24) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:196) [clock_data_publisher.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/publisher/data/clock_data_publisher.cpp:10) |
| `joy_cmd_vel_topic` | `geometry_msgs/Twist` | 仿真桥订阅 | `/joy/cmd_vel` | 全局遥控速度输入入口。通常由手柄映射节点或遥控节点发布，最终进入仿真内部速度命令链路。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:40) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:258) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:297) [cmd_vel_command_subscriber.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/subscriber/data/cmd_vel_command_subscriber.cpp:10) |
| `cmd_vel_topic` | `geometry_msgs/Twist` | 仿真桥订阅 | `/scout1/cmd_vel` | 车辆本地速度输入入口。算法节点、导航节点、脚本控制通常直接向这个 topic 发送线速度和角速度命令。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:39) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:258) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:295) [cmd_vel_command_subscriber.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/subscriber/data/cmd_vel_command_subscriber.cpp:10) |
| `imu_topic` | `sensor_msgs/Imu` | 仿真桥发布 | `/scout1/imu/data` | 发布车辆 IMU 姿态、角速度、线加速度，用于状态估计、融合、可视化。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:42) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:194) [imu_data_publisher.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/publisher/data/imu_data_publisher.cpp:10) |
| `lidar sensor topic` | `sensor_msgs/PointCloud2` | 仿真桥发布 | `/scout1/lidar/points` | 发布雷达点云。当前来自 `lidar16` 传感器，供建图、避障、目标感知等算法使用。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:52) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:331) [lidar_data_publisher.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/publisher/data/lidar_data_publisher.cpp:15) |
| `odom_topic` | `nav_msgs/Odometry` | 仿真桥发布 | `/scout1/odom` | 发布车辆里程计状态，包括位置、姿态、线速度、角速度，是定位、控制和可视化的基础状态输出。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:41) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:192) [odom_data_publisher.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/publisher/data/odom_data_publisher.cpp:10) |
| `robot_mode_topic` | `std_msgs/String` | 仿真桥发布 | `/scout1/teleop/mode` | 旧版机器人模式输出，内容是 JSON 字符串，包含顶层模式、子状态、是否接受运动命令。主要用于兼容已有消费方。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:47) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:313) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:570) |
| `robot_mode_structured_topic` | `ausim_msg/RobotMode` | 仿真桥发布 | `/scout1/teleop/mode_structured` | 新版结构化模式输出。除模式状态外，还携带 `vehicle_id`、`ros_namespace`、最近一次离散命令的序号与回执状态，适合作为正式程序接口。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:48) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:204) [robot_mode_publisher.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/publisher/semantic/robot_mode_publisher.cpp:9) [robot_mode.cpp](/home/x/mujoco/ausim2/ausim_common/src/converts/ausim_msg/robot_mode.cpp:18) |
| `tf` | `tf2_msgs/TFMessage` | 仿真桥发布 | `/tf` | 发布动态坐标变换。当前主要包含车辆主刚体的动态位姿，例如 `odom -> scout1/base_link`。 | [sim_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/sim_config.yaml:24) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:200) [transform_data_publisher.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/publisher/data/transform_data_publisher.cpp:10) |
| `tf_static` | `tf2_msgs/TFMessage` | 仿真桥发布 | `/tf_static` | 发布静态坐标变换。当前主要包含传感器到车体的固定外参，例如 `scout1/base_link -> scout1/lidar_link`。 | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:58) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:264) |

## Service 对照表

当前 `scout1` 配置通过 `joy_action_services` 暴露了两个业务 service，统一类型为 `std_srvs/Trigger`。  
服务名本身写成绝对路径，因此是全局 service，不挂在 `/scout1` namespace 下。

| Service | 类型 | 方向 | 当前实例 | 作用 | 内部事件 | 代码/配置来源 |
| --- | --- | --- | --- | --- | --- | --- |
| `joy_action_services[0]` | `std_srvs/Trigger` | 仿真桥提供 | `/joy/action3` | 向仿真发送“重置”离散事件。调用成功后，仿真桥会等待离散命令回执并返回 `success/message`。 | `reset` | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:44) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:304) |
| `joy_action_services[1]` | `std_srvs/Trigger` | 仿真桥提供 | `/joy/action4` | 向仿真发送“急停”离散事件。调用后会进入机器人模式状态机或离散事件处理链路，并返回执行回执。 | `estop` | [scout_v2_config.yaml](/home/x/mujoco/ausim2/ground_vehicle/cfg/robot/scout_v2_config.yaml:45) [ros2_bridge.cpp](/home/x/mujoco/ausim2/ausim_common/src/ros/ros2_bridge.cpp:304) |

## 业务链路简述

### 1. 控制输入链路

- `/joy/cmd_vel` 或 `/scout1/cmd_vel`
- `-> CmdVelCommandSubscriber`
- `-> internal CmdVelData`
- `-> ipc::VelocityCommandPacket`
- `-> 仿真控制器`
- `-> Mujoco / 车辆运动`

### 2. 状态输出链路

- 仿真状态
- `-> TelemetryPacket / LidarPacket`
- `-> ROS bridge`
- `-> /scout1/odom`
- `-> /scout1/imu/data`
- `-> /clock`
- `-> /tf`
- `-> /scout1/teleop/mode`
- `-> /scout1/teleop/mode_structured`
- `-> /scout1/lidar/points`

## 备注

- `teleop/mode` 是兼容旧程序的字符串接口，新增程序优先使用 `teleop/mode_structured`。
- `tf` 和 `tf_static` 虽然是 ROS 标准总线，但这里记录它们，是因为它们承载的是当前仿真桥主动发布的业务坐标关系。
- 如果后续更换机器人型号或修改 YAML，本文应以对应 robot config 为准重新整理。
