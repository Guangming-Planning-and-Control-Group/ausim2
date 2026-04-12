# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指引。

## 构建

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build -j
```

MuJoCo 使用仓内 vendored release 包：

```bash
third_party/mujoco-3.6.0/include/mujoco
third_party/mujoco-3.6.0/lib/libmujoco.so.3.6.0
third_party/mujoco-3.6.0/bin/mujoco_plugin/
```

产物：`build/bin/quadrotor` 和 `build/bin/quadrotor_ros_bridge`。

## 运行

```bash
./build/bin/quadrotor --headless
./build/bin/quadrotor --sim-config ./quadrotor/cfg/sim_config.yaml \
  --robot-config ./quadrotor/cfg/robot/xi35_config.yaml --headless
```

ROS2 速度指令：

```bash
ros2 topic pub /uav1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.3}}" --once
```

## 架构

两个进程通过本地 `socketpair` 通信：

- `quadrotor` — 仿真 + 控制循环
- `quadrotor_ros_bridge` — ROS2 I/O（子进程）

交换两个 DataBoard 槽位：
- `runtime.velocity_command`（bridge → sim）
- `runtime.telemetry_snapshot`（sim → bridge）

### 各层职责

| 层 | 路径 | 职责 |
|----|------|------|
| App/Manager | `src/app/`, `src/manager/` | 启动、子进程生命周期、IPC 线程 |
| Sim | `src/sim/` | MuJoCo bindings、状态读取、执行器写回、viewer 循环 |
| Runtime | `src/runtime/` | `VehicleRuntime` 驱动 SE3 控制器；`GoalProvider` 选择目标来源；`DataBoardInterface` 持有两个共享槽位 |
| Controller | `src/controller/`, `src/control/`, `src/math/` | SE3 控制律、MotorMixer、姿态几何——不依赖 ROS 和 MuJoCo 名称 |
| Data / Converts | `src/data/`, `src/converts/` | 中立消息结构体；内部 ↔ ROS2 ↔ IPC 包格式转换 |
| ROS Bridge | `src/ros/` | ROS2 节点、publisher/subscriber 管理、IPC 收发 |
| Config | `src/config/` | YAML 解析到 `QuadrotorConfig` |

### ROS Bridge 扩展点

`RosBridgeProcess` 通过两个接口 vector 管理所有 pub/sub：

- `ITelemetryPublisher`（`ros/publisher/i_telemetry_publisher.hpp`）：接收 `TelemetryPacket`，内部完成转换和发布
- `ICommandSubscriber`（`ros/subscriber/i_command_subscriber.hpp`）：回调在构造时注入

新增传感器发布者：继承 `ITelemetryPublisher`，在 `ros2_bridge.cpp` 的 `sensors[]` 循环加一个 `else if` 分支即可，不需要改 `RosBridgeProcess` 其他部分。

### DataBoard

`src/common/db/DataBoard` 是以 `std::any` 为底层的单例键值存储，用 `SecurityData<T>` 包装保证线程安全读写。所有跨层数据交换都经过它。

### 配置加载顺序

1. 先加载 `sim_config.yaml`（全局仿真参数、bridge 参数、`robot_config` 路径）
2. `robot_config` 字段指向单机型 YAML（bindings、控制器、话题、传感器）

`robot_config` 关键字段：
- `simulation.control_mode`：`1` = 速度控制，`2` = 位置控制
- `simulation.example_mode`：`0` = ROS cmd_vel，`1` = 简单目标，`2` = 圆轨迹 demo

### 构建目标

| 目标 | 类型 | 内容 |
|------|------|------|
| `quadrotor_mujoco_viewer` | 静态库 | third_party MuJoCo GLFW viewer |
| `quadrotor_sim_core` | 静态库 | 除 ROS 外的全部内容（sim、runtime、controller、config、IPC） |
| `quadrotor_ros_bridge_core` | 静态库 | ROS2 bridge + converts + config |
| `quadrotor` | 可执行文件 | 链接 `quadrotor_sim_core` |
| `quadrotor_ros_bridge` | 可执行文件 | 链接 `quadrotor_ros_bridge_core` |

`quadrotor_sim_core` 无 ROS 依赖；`quadrotor_ros_bridge_core` 无 MuJoCo 依赖。

## 新增机型

复制 `cfg/robot/<name>_config.yaml`，修改 `identity`、`model`、`bindings`、`vehicle`、`controller`，然后将 `sim_config.yaml` 的 `robot_config` 指向新文件。

## 新增传感器

1. 在 MJCF 里添加 sensor / camera / site
2. 在 `cfg/robot/<name>_config.yaml` 的 `sensors[]` 中声明
3. 在 `src/sim/` 添加读取逻辑（如需要）
4. 在 `src/ros/publisher/data/` 新建 publisher，继承 `ITelemetryPublisher`
5. 在 `ros2_bridge.cpp` 的 `sensors[]` 循环加 `else if` 分支实例化
