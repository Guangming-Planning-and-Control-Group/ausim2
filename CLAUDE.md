# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指引。

## 仓库组成

`ausim2` 是基于 MuJoCo 3.6.0 的多平台仿真仓库，当前包含两个并列的仿真工程与一个共享的公共层：

- `quadrotor/` — 四旋翼仿真（产物 `build/bin/quadrotor` + `build/bin/quadrotor_ros_bridge`）
- `ground_vehicle/` — Scout 等地面车仿真（产物 `build/bin/scout` + `build/bin/ausim_ros_bridge`）
- `ausim_common/` — 两个工程共享的 config / IPC / ROS bridge / 状态机等代码
- `assets/` — MJCF 模型与网格
- `third_party/` — vendored MuJoCo、ray-caster 传感器插件、`ausim_msg` ROS 接口、`dynamic_obs_generator`、`remote_control` 等
- `build.sh` / `em_run.sh` — 统一构建与启动脚本

MuJoCo 已完全内置（**无需手动安装**）：

```
third_party/mujoco-3.6.0/include/mujoco
third_party/mujoco-3.6.0/lib/libmujoco.so.3.6.0
third_party/mujoco-3.6.0/bin/mujoco_plugin/
build/bin/mujoco_plugin/libsensor_raycaster.so   # 仓内自研插件
```

## 构建

```bash
./build.sh
```

脚本会自动：

1. 通过 `apt` 补齐基础依赖（`build-essential` / `cmake` / `libeigen3-dev` / `libyaml-cpp-dev` / `libglfw3-dev`）
2. 检查 ROS2 Humble（**不会自动安装**，需预先 `/opt/ros/humble/setup.bash` 可用）
3. 用 `colcon` 构建 `third_party/ausim_msg` overlay 到 `build/ros_ws/install`
4. 在 `build/` 下执行 `cmake` 配置与并行编译

主要产物：`quadrotor`、`scout`、`quadrotor_ros_bridge`、`ausim_ros_bridge`、`libsensor_raycaster.so`。

MuJoCo 插件 smoke test（可选）：`./build/bin/mujoco_ray_caster_smoketest`。

## 运行

推荐 `./em_run.sh`：

```bash
./em_run.sh --headless         # 无 GUI
./em_run.sh --viewer           # 带 MuJoCo viewer
./em_run.sh -S --headless      # 重新选择仿真目标（如 omnidrone / crazyfile / xi35 / scout_v2）
```

`em_run.sh` 行为：

- 首次运行提示选择 registry 中的机型 ID，并记入 `.em_run_target`
- 自动导出 `MUJOCO_PLUGIN_DIR`（同时覆盖仓内 vendored 目录与 `build/bin/mujoco_plugin`）
- 若所选机型在 registry 启用动态障碍，会先调 `third_party/dynamic_obs_generator/generate_scene_obstacles.py` 生成场景
- 支持透传 `--sim-config` / `--robot-config` / `--headless` / `--viewer`

默认配置：

- 四旋翼：`quadrotor/cfg/sim_config.yaml` → `quadrotor/cfg/robot/crazyfile_config.yaml`
- 地面车：`ground_vehicle/cfg/sim_config.yaml` → `ground_vehicle/cfg/robot/scout_v2_config.yaml`

在其他 shell 用 ROS2 命令前记得 `source /opt/ros/humble/setup.bash`。

## 架构（两工程同构）

两个工程的进程模型一致：仿真/控制进程通过本地 `socketpair` 与 ROS bridge 子进程通信，交换两个 DataBoard 槽位：

- `runtime.velocity_command`（bridge → sim，来自 `/cmd_vel`）
- `runtime.telemetry_snapshot`（sim → bridge，驱动所有 publisher）

### 分层职责

| 层 | 路径 | 职责 |
|----|------|------|
| App/Manager | `<proj>/src/app/`, `<proj>/src/manager/`（四旋翼）/ `ground_vehicle/src/main.cpp` | 启动、子进程生命周期、IPC 线程 |
| Sim | `<proj>/src/sim/` | MuJoCo bindings、状态读取、执行器写回、viewer 循环 |
| Runtime | `ausim_common/` 中的 `VehicleRuntime` / `GoalProvider` / `DataBoardInterface` | 状态机驱动、目标来源选择、跨层数据槽位 |
| Controller / Control | `quadrotor/src/controller/`、`quadrotor/src/control/`、`quadrotor/src/math/`；`ground_vehicle/src/control/` | SE3 控制律 + MotorMixer（四旋翼）/ 差速驱动（Scout），不依赖 ROS、MuJoCo 名称 |
| Data / Converts | `src/data/`, `src/converts/` | 中立消息结构体；内部 ↔ ROS2 ↔ IPC 包格式转换 |
| ROS Bridge | `src/ros/`（或 `ausim_common/`） | ROS2 节点、publisher/subscriber 管理、IPC 收发 |
| Config | `src/config/` | YAML 解析到 `QuadrotorConfig` / `GroundVehicleConfig` |

### DataBoard

`ausim_common/common/db/DataBoard` 是以 `std::any` 为底层的单例键值存储，用 `SecurityData<T>` 包装保证线程安全读写。所有跨层数据交换都经过它。

### ROS Bridge 扩展点

`RosBridgeProcess` 通过两个接口 vector 管理所有 pub/sub：

- `ITelemetryPublisher`（`ros/publisher/i_telemetry_publisher.hpp`）：接收 `TelemetryPacket`，内部完成转换和发布
- `ICommandSubscriber`（`ros/subscriber/i_command_subscriber.hpp`）：回调在构造时注入

新增传感器发布者：继承 `ITelemetryPublisher`，在 `ros2_bridge.cpp` 的 `sensors[]` 循环加一个 `else if` 分支即可，无需改 `RosBridgeProcess` 其他部分。

### 配置加载顺序

1. 先加载 `sim_config.yaml`（全局仿真参数、bridge 参数、`robot_config` 路径）
2. `robot_config` 指向单机型 YAML（bindings、vehicle、controller、话题、传感器、`mode_machine`）
3. `mode_machine` 可内联或指向 `cfg/teleop/<name>.yaml`

四旋翼 `sim_config.yaml` 关键字段：
- `simulation.control_mode`：`1` = 速度控制，`2` = 位置控制
- `simulation.example_mode`：`0` = ROS cmd_vel，`1` = 简单目标，`2` = 圆轨迹 demo

## Teleop 状态机

两工程共用同一状态机引擎（分层状态 + event / condition / timeout 三种触发器 + action 注册表）。action 名字由 `GoalProvider` 通过 `ModeActionsRegistry::RegisterAction` 注入，YAML 只认名字——新增 action 不需要改 C++ 分发代码。

- 四旋翼默认 `quadrotor/cfg/teleop/quadrotor_default.yaml`：`on_ground` → (takeoff) → `hover` ⇄ `velocity_control`，60s 无事件自动 `land`，`{any} --estop--> estop`
- 地面车默认 `ground_vehicle/cfg/teleop/scout_default.yaml`：`stopped` ⇄ `manual_drive`（由 `motion_active/inactive` 驱动），`{any} --estop--> estop`

`accepts_motion: false` 的子状态会过滤 `/cmd_vel`——这是"必须先 takeoff 才能飞"的实现方式。离散动作通过 `/joy/actionN`（`std_srvs/Trigger`）或 `third_party/remote_control` 触发。

## 相机与深度

四旋翼 `sensors[]` 中 `type: camera` 支持附带 `depth:` 子块，RGB 与深度走同一条内部链路：

- `depth.enabled: false` 时不向 data board / IPC / ROS image publisher 注入深度流
- 深度分辨率继承 MJCF 相机分辨率，运行时覆盖 ray-caster `size`（MJCF 的 `size` 需不小于运行分辨率）
- `depth.data_type` 覆盖 ray-caster 的 `sensor_data_types`，ROS depth image 当前支持单标量类型（如 `distance_to_image_plane_inf_zero`、`data_inf_zero`、`normal` 等；`pos_w/pos_b` 是三通道流，需单独处理）
- `depth.compute_rate_hz` 控制 ray-caster 实际计算频率，会自动换算为 physics step 间隔
- `depth.worker_threads` 映射到 `mujoco_ray_caster` 的 `num_thread`，目前上游多线程实现不稳定，建议保持 `0`

## 构建目标

| 目标 | 类型 | 内容 |
|------|------|------|
| `quadrotor_mujoco_viewer` | 静态库 | third_party MuJoCo GLFW viewer |
| `quadrotor_sim_core` | 静态库 | 四旋翼除 ROS 外的全部内容（sim、runtime、controller、config、IPC） |
| `quadrotor_ros_bridge_core` | 静态库 | 四旋翼 ROS2 bridge + converts + config |
| `quadrotor` / `scout` | 可执行文件 | 仿真主程序，链接对应 `*_sim_core` |
| `quadrotor_ros_bridge` / `ausim_ros_bridge` | 可执行文件 | ROS2 bridge 子进程 |
| `sensor_raycaster` | MuJoCo 插件 | 深度/激光 ray-caster，产物落在 `build/bin/mujoco_plugin/` |

`*_sim_core` 无 ROS 依赖；`*_ros_bridge_core` 无 MuJoCo 依赖。

## 新增机型

复制 `cfg/robot/<name>_config.yaml`，修改 `identity`、`model`、`bindings`、`vehicle`/`ground_vehicle`、`controller`、`mode_machine`，然后把对应 `sim_config.yaml` 的 `robot_config` 指向新文件。如需同时在 `em_run.sh` 列表中出现，还要在启动脚本的 registry 中注册该机型 ID。

## 新增传感器（四旋翼）

1. 在 MJCF 中添加 sensor / camera / site
2. 在 `cfg/robot/<name>_config.yaml` 的 `sensors[]` 声明（`enabled` / `type` / `topic` / `frame_id` / `rate_hz`）
3. 必要时在 `src/sim/` 加读取逻辑
4. 在 `src/ros/publisher/data/` 新建 `<type>_data_publisher.hpp/.cpp`，继承 `ITelemetryPublisher`
5. 在 `ros2_bridge.cpp` 的 `sensors[]` 循环里为新 `sensor.type` 加一个 `else if` 分支

控制器和混控层无需改动。

## 模块文档导航

- [README.md](README.md) — 仓库总览、构建与启动入口
- [quadrotor/README.md](quadrotor/README.md) — 四旋翼分层、配置、状态机、传感器扩展
- [ground_vehicle/README.md](ground_vehicle/README.md) — 地面车分层、配置、状态机
- [third_party/README.md](third_party/README.md) — vendored / 外部组件总览
- [third_party/mujoco_ray_caster/README.zh-CN.md](third_party/mujoco_ray_caster/README.zh-CN.md) — ray-caster 插件
- [third_party/dynamic_obs_generator/README.md](third_party/dynamic_obs_generator/README.md) — 动态障碍生成器
- [third_party/remote_control/README.md](third_party/remote_control/README.md) — 手柄 / 键盘遥控
