# Quadrotor C++ Runtime

基于 MuJoCo 的四旋翼仿真工程。代码按 `config / data / converts / controller / runtime / sim / ros` 分层，ROS2 bridge 作为独立子进程运行，通过 IPC 与仿真进程交换数据。

构建 (`./build.sh`) 与启动 (`./em_run.sh`) 见仓库根 [README](../README.md)；本文件只描述 quadrotor 自身的结构、配置与状态机。

产物：`build/bin/quadrotor`、`build/bin/quadrotor_ros_bridge`、`build/bin/mujoco_plugin/libsensor_raycaster.so`。运行时会优先读取 `MUJOCO_PLUGIN_DIR`；`em_run.sh` 会自动导出。

## 进程模型

两个进程通过本地 `socketpair` 通信：

```
quadrotor (仿真 + 控制)  <──IPC──>  quadrotor_ros_bridge (ROS2 I/O)
```

交换的两个数据槽：
- `runtime.velocity_command`：bridge → sim，来自 `/cmd_vel`
- `runtime.telemetry_snapshot`：sim → bridge，驱动所有发布者

## 分层职责

| 层 | 路径 | 职责 |
|----|------|------|
| App/Manager | `src/app/`, `src/manager/` | 启动、子进程生命周期、IPC 线程 |
| Sim | `src/sim/` | MuJoCo bindings、状态读取、执行器写回、viewer 循环 |
| Runtime | `src/runtime/` | `VehicleRuntime` 驱动 SE3 控制器；`GoalProvider` 选择目标来源 |
| Controller | `src/controller/`, `src/control/`, `src/math/` | SE3 控制律、MotorMixer、姿态几何——不依赖 ROS 和 MuJoCo 名称 |
| Data / Converts | `src/data/`, `src/converts/` | 中立消息结构体；内部 ↔ ROS2 ↔ IPC 包格式转换 |
| ROS Bridge | `src/ros/` | ROS2 节点、publisher/subscriber 管理、IPC 收发 |
| Config | `src/config/` | YAML 解析到 `QuadrotorConfig` |

## ROS Bridge 架构

`RosBridgeProcess` 通过两个接口 vector 管理所有 pub/sub：

```cpp
std::vector<std::unique_ptr<ITelemetryPublisher>> publishers_;
std::vector<std::unique_ptr<ICommandSubscriber>>  subscribers_;
```

- `ITelemetryPublisher`（`ros/publisher/i_telemetry_publisher.hpp`）：所有遥测发布者的基类，接收 `TelemetryPacket`，内部完成转换和发布
- `ICommandSubscriber`（`ros/subscriber/i_command_subscriber.hpp`）：所有指令订阅者的基类，回调在构造时注入

`PublishTelemetry()` 只做一件事：

```cpp
for (auto& pub : publishers_) pub->Publish(*packet);
```

`sensors[]` 配置循环驱动额外 publisher 的实例化，新增传感器类型只需在该循环加一个 `else if` 分支。

## 配置

### `cfg/sim_config.yaml`

全局仿真参数和 bridge 参数，以及要加载的机型配置路径：

```yaml
robot_config: robot/crazyfile_config.yaml

simulation:
  duration: 0.0
  dt: 0.001
  control_mode: 2
  example_mode: 0

goal:
  position: [0.0, 0.0, 0.3]
  velocity: [0.5, 0.0, 0.0]
  heading: [1.0, 0.0, 0.0]

trajectory:
  wait_time: 1.5
  height: 0.3
  radius: 0.5
  speed_hz: 0.3
  height_gain: 1.5

viewer:
  enabled: true
  show_mode_state_overlay: false

ros2:
  node_name: sim_bridge
  publish_rate_hz: 100.0
  command_timeout: 0.5
```

### `cfg/robot/<name>_config.yaml`

机型相关参数：

```yaml
identity:
  vehicle_id: cf2
  namespace: /uav1
  frame_prefix: uav1

model:
  scene_xml: ../../../assets/crazyfile/scene.xml
  body_name: cf2
  aircraft_forward_axis: [0.0, 1.0, 0.0]

# teleop / 状态机，详见"Teleop 状态机"章节
mode_machine: ../teleop/quadrotor_default.yaml
```

`control_mode` / `example_mode` / `goal` / `trajectory` 现在放在 `cfg/sim_config.yaml` 里，作为全局仿真配置。

`viewer.show_mode_state_overlay: true` 时，会在 MuJoCo viewer 右上角叠加显示当前状态机子状态，例如 `on_ground` / `hover`。

`model.aircraft_forward_axis` 表示"飞机机头在模型 body 坐标系里的方向"。Crazyflie 约定：
- 模型 `+x`：飞机右侧，模型 `+y`：飞机机头，故为 `[0.0, 1.0, 0.0]`
- 影响速度模式下的"当前 heading"计算和姿态保持时的机头朝向约束

`control_mode: 1` 下，`/cmd_vel` 的语义：
- 无新命令时：锁定当前位置与当前朝向，进入悬停保持
- `linear.x / linear.y`：无人机当前局部水平坐标系的前 / 左方向速度（与地面平行）
- `linear.z`：竖直速度
- `angular.z`：机体系偏航角速度；持续发 `0` 时保持当前朝向

### `cfg/teleop/<name>.yaml`

分层状态机 + action 参数的定义，可由多个机型共享。默认提供 `quadrotor_default.yaml`，详见下节。

## ROS2 话题

默认配置（namespace `/uav1`）：

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/uav1/cmd_vel` | `geometry_msgs/Twist` |
| 订阅 | `/joy/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/uav1/odom` | `nav_msgs/Odometry` |
| 发布 | `/uav1/imu/data` | `sensor_msgs/Imu` |
| 发布 | `/uav1/camera/image_raw` | `sensor_msgs/Image` (`rgb8`) |
| 发布 | `/uav1/camera/depth/image_raw` | `sensor_msgs/Image` (`32FC1`) |
| 发布 | `/uav1/teleop/mode` | `std_msgs/String` |
| 发布 | `/clock` | `rosgraph_msgs/Clock` |
| TF | `uav1/odom → uav1/base_link` | — |

默认配置下还会注册以下离散命令服务：

| 方向 | 话题 | 类型 |
|------|------|------|
| 服务 | `/joy/action1` | `std_srvs/Trigger` |
| 服务 | `/joy/action2` | `std_srvs/Trigger` |
| 服务 | `/joy/action3` | `std_srvs/Trigger` |
| 服务 | `/joy/action4` | `std_srvs/Trigger` |

## Teleop 状态机

四旋翼共用 `cfg/teleop/quadrotor_default.yaml` 定义的分层状态机。机型 YAML 通过一行引用即可：

```yaml
mode_machine: ../teleop/quadrotor_default.yaml
```

### 默认状态

| Sub-state          | Top-level       | accepts_motion (cmd_vel) |
|--------------------|-----------------|:------------------------:|
| `on_ground`        | `SAFE`          | false |
| `hover`            | `MANUAL_READY`  | true  |
| `velocity_control` | `MANUAL_ACTIVE` | true  |
| `estop`            | `FAULT`         | false |

`accepts_motion: false` 的子状态会**过滤掉** `/cmd_vel`——这也是"必须先调 takeoff 才能飞"的实现方式。

### 转移触发器

每条 transition 恰好使用一种触发：

| 触发 | YAML 字段 | 语义 |
|------|-----------|------|
| 事件 | `event: <name>` | 任意字符串（`takeoff` / `land` / `estop` / 自定义），由 `/joy/actionN` service 或其他离散命令入口产生 |
| 条件 | `condition: motion_active` / `motion_inactive` | 由 `/cmd_vel` 活跃/超时驱动 |
| 超时 | `timeout: <sec>` | 进入 `from` 后无转移经过 N 秒自动触发 |

Event-driven 转移**忽略** `condition`；加载时会对同时设置两者的条目打 warning。

### 默认转移

```
on_ground        --event:takeoff--------> hover               (action: takeoff)
hover            --event:land-----------> on_ground           (action: land)
hover            --condition:motion_act-> velocity_control
velocity_control --condition:motion_ina-> hover
hover            --timeout:60s----------> on_ground           (action: land)
{any}            --event:estop---------> estop               (action: emergency_stop)
```

### Action 注册表

状态机只认 action 名字，回调由 `GoalProvider` 在构造时向 `ModeActionsRegistry` 注册：

| action name       | 参数来源 (`actions:` 子节) | 默认行为 |
|-------------------|----------------------------|----------|
| `takeoff`         | `actions.takeoff.height`   | 跳到悬停高度（`climb_rate=0` 即瞬时） |
| `land`            | `actions.land.descent_rate`| 落回出生点高度（`descent_rate=0` 即瞬时） |
| `emergency_stop`  | —                          | 骨架占位 |

自定义 action 只需 `provider.RegisterAction("my_action", cb)`，再在 YAML 里用 `action: my_action`——无需改 C++ 分发代码。

### 触发途径

**ROS2 service**（默认 RC 动作入口）：

```bash
ros2 service call /joy/action1 std_srvs/srv/Trigger "{}"  # takeoff
ros2 service call /joy/action2 std_srvs/srv/Trigger "{}"  # land
ros2 service call /joy/action3 std_srvs/srv/Trigger "{}"  # reset
ros2 service call /joy/action4 std_srvs/srv/Trigger "{}"  # estop
```

**手柄 / 键盘**（`third_party/remote_control`）：默认把摇杆输出到 `/joy/cmd_vel`，并把离散动作槽位调用到 `/joy/actionN`，详见 `third_party/remote_control/README.md`。

**速度指令**：

```bash
ros2 topic pub /uav1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.3}}" --once
```

注意：初始态 `on_ground.accepts_motion=false` 会过滤 `/cmd_vel`，需先调 `/joy/action1` 让状态机进入 `hover`。

### 自定义机型状态机

在 `cfg/teleop/` 下新增 YAML（如 `xi35_aggressive.yaml`），把机型配置的 `mode_machine:` 指向它即可。字段支持内联 map 或字符串路径两种形式。

## 新增机型

复制 `cfg/robot/<name>_config.yaml`，修改 `identity`、`model`、`bindings`、`vehicle`、`controller`，然后将 `sim_config.yaml` 的 `robot_config` 指向新文件。

## 新增传感器

1. 在 MJCF 里添加 sensor / camera / site
2. 在 `cfg/robot/<name>_config.yaml` 的 `sensors[]` 中声明（`enabled: true`、`type`、`topic`、`frame_id`）
3. 在 `src/sim/` 添加读取逻辑（如需要）
4. 在 `src/ros/publisher/data/` 新建 `<type>_data_publisher.hpp/.cpp`，继承 `ITelemetryPublisher`，构造函数接收 `(node, topic, frame_id)`，`Publish(const ipc::TelemetryPacket&)` 内部完成转换和发布
5. 在 `ros2_bridge.cpp` 的 `sensors[]` 循环里为新 `sensor.type` 加一个 `else if` 分支

控制器和混控层无需改动。

### 相机深度流

`type: camera` 支持在同一个传感器配置下附带 `depth:` 子块。RGB 和 depth 会沿同一条内部链路流动：

```yaml
sensors:
  - name: front_camera
    type: camera
    enabled: true
    frame_id: camera_link
    topic: camera/image_raw
    rate_hz: 30.0
    depth:
      enabled: true
      frame_id: camera_link
      topic: camera/depth/image_raw
      data_type: distance_to_image_plane_inf_zero
      rate_hz: 30.0
      compute_rate_hz: 30.0
      worker_threads: 0
```

- `depth.enabled: false` 时不会向 data board、IPC 和 ROS image publisher 注入深度流
- 相机名、depth plugin 传感器名和分辨率都从 MJCF/XML 读取，robot config 只保留发布相关字段
- depth 分辨率继承 MJCF 相机分辨率，运行时会覆盖 ray-caster `size`；MJCF 中的 `size` 仍需提供不小于运行分辨率的容量
- `depth.data_type` 会覆盖 ray-caster 的 `sensor_data_types`，当前 ROS depth image 路径支持单个标量类型，例如 `data_inf_zero`、`normal`、`distance_to_image_plane_inf_zero`、`image_plane_image_inf_zero`、`image_plane_normal_inf_zero`；`pos_w/pos_b` 是每像素 xyz 三通道数据，需要单独的数据流
- `depth.compute_rate_hz` 用来控制 ray-caster 实际计算频率；如果物理步长是 `0.001` 且该值为 `30`，插件会自动写成约每 `33` 个 physics step 更新一次
- `depth.worker_threads` 会映射到 `mujoco_ray_caster` 的 `num_thread` 配置，只作用于 depth ray-caster，不会改动 RGB 的 OpenGL 渲染线程模型；当前上游多线程实现不稳定，默认建议保持 `0`
- 当前 Crazyflie 示例会自动把 `front_camera` 绑定到 `assets/crazyfile/cf2.xml` 里的第一个非 track 相机，并把 depth 流绑定到同一相机上的 ray-caster plugin
