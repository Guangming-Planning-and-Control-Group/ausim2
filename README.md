# ausim2

## 1. 介绍仓库

`ausim2` 是一个基于 MuJoCo 3.6.0 的仿真仓库，当前核心可执行程序包括：

- `build/bin/quadrotor`：四旋翼仿真主程序，集成 ray-caster 深度传感器插件与动态障碍场景生成。
- `build/bin/scout`：地面车辆仿真主程序（源码位于 `ground_vehicle/`）。
- `build/bin/quadrotor_ros_bridge` / `build/bin/ausim_ros_bridge`：ROS2 桥接程序。

**MuJoCo 依赖已完全内置**，不需要手动编译或安装；仓内 vendored 路径：

- `third_party/mujoco-3.6.0/include/mujoco`：MuJoCo 公开头文件
- `third_party/mujoco-3.6.0/lib/libmujoco.so.3.6.0`：MuJoCo 运行时库
- `third_party/mujoco-3.6.0/bin/mujoco_plugin/`：MuJoCo 官方插件与 OBJ/STL decoder
- `build/bin/mujoco_plugin/libsensor_raycaster.so`：本仓库编译出的自定义传感器插件

与运行直接相关的目录：

- `assets/`：MJCF 模型、网格与场景文件
- `quadrotor/`：四旋翼仿真源码与配置（见 [quadrotor/README.md](quadrotor/README.md)）
- `ground_vehicle/`：Scout 仿真源码与配置（见 [ground_vehicle/README.md](ground_vehicle/README.md)）
- `ausim_common/`：配置、IPC、ROS2 bridge、状态机等公共代码
- `third_party/`：MuJoCo release 包、ray-caster 插件源码、动态障碍生成器、remote_control
- `build.sh` / `em_run.sh`：统一构建与启动脚本

## 2. 编译

直接运行：

```bash
./build.sh
```

脚本会自动：

1. 检查并通过 `apt` 安装常规依赖（`build-essential`、`cmake`、`libeigen3-dev`、`libyaml-cpp-dev`、`libglfw3-dev`）
2. 检查 ROS2 Humble 是否已安装（**ROS 不会自动装**，缺失时提示并退出）
3. 构建 ROS 接口 overlay（`third_party/ausim_msg`）到 `build/ros_ws/install`
4. 在仓库根 `build/` 下执行 `cmake` 配置与并行编译

主要产物：

- `build/bin/quadrotor`
- `build/bin/scout`
- `build/bin/quadrotor_ros_bridge`
- `build/bin/ausim_ros_bridge`
- `build/bin/mujoco_plugin/libsensor_raycaster.so`

系统预置要求：ROS2 Humble（`/opt/ros/humble/setup.bash`）、`colcon`、`apt`（Ubuntu 22.04 系）；其他依赖由脚本负责。

可选的 MuJoCo 插件 smoke test：

```bash
./build/bin/mujoco_ray_caster_smoketest
```

## 3. 启动

推荐使用统一启动脚本 `./em_run.sh`：

```bash
./em_run.sh --headless     # 无 GUI
./em_run.sh --viewer       # 带 MuJoCo viewer
./em_run.sh -S --headless  # 重新选择仿真目标（例如 omnidrone / crazyfile / xi35 / scout_v2）
```

`em_run.sh` 的行为：

- 首次运行提示选择仿真目标（来自 registry 的模型 ID，如 `omnidrone`、`crazyfile`、`xi35`、`scout_v2`），并保存到 `.em_run_target`
- 自动导出 `MUJOCO_PLUGIN_DIR`（同时覆盖仓内 vendored 目录与 `build/bin/mujoco_plugin`）
- 若所选模型在 registry 中开启动态障碍，会先调用 `third_party/dynamic_obs_generator/generate_scene_obstacles.py` 生成场景
- 支持透传 `--sim-config` / `--robot-config` / `--headless` / `--viewer` 等参数

默认配置文件：

- 四旋翼默认：`quadrotor/cfg/sim_config.yaml` → `quadrotor/cfg/robot/crazyfile_config.yaml`
- 地面车辆默认：`ground_vehicle/cfg/sim_config.yaml` → `ground_vehicle/cfg/robot/scout_v2_config.yaml`

如需在另一个 shell 中使用 ROS2 命令，请先：

```bash
source /opt/ros/humble/setup.bash
```

## 4. 模块文档

- [quadrotor/README.md](quadrotor/README.md)：四旋翼分层结构、配置、状态机（teleop）、传感器扩展
- [ground_vehicle/README.md](ground_vehicle/README.md)：Scout 分层结构、配置、状态机
- [third_party/README.md](third_party/README.md)：vendored / 外部组件说明

## 5. 仓库内 README 导航（含作用）

说明：以下列表按当前仓库文件扫描整理，覆盖源码、第三方与参考资料中的 README。

- [README.md](README.md)：仓库总览、构建与启动入口说明
- [quadrotor/README.md](quadrotor/README.md)：四旋翼模块架构、配置和扩展说明
- [ground_vehicle/README.md](ground_vehicle/README.md)：地面车辆模块架构、配置和扩展说明
- [assets/crazyfile/README.md](assets/crazyfile/README.md)：Crazyfile 资产与模型说明
- [third_party/README.md](third_party/README.md)：第三方组件总览与来源说明
- [third_party/ausim_msg/README.md](third_party/ausim_msg/README.md)：语义消息包说明
- [build/ros_ws/install/share/ausim_msg/README.md](build/ros_ws/install/share/ausim_msg/README.md)：`ausim_msg` 安装后导出的 README（构建产物）
- [third_party/dynamic_obs_generator/README.md](third_party/dynamic_obs_generator/README.md)：动态障碍场景生成器说明
- [third_party/remote_control/README.md](third_party/remote_control/README.md)：遥控组件说明
- [third_party/mujoco_ray_caster/README.zh-CN.md](third_party/mujoco_ray_caster/README.zh-CN.md)：MuJoCo ray-caster 插件中文说明
- [third_party/mujoco-3.6.0/simulate/README.md](third_party/mujoco-3.6.0/simulate/README.md)：MuJoCo simulate 示例程序说明
- [third_party/mujoco-3.6.0/model/adhesion/README.md](third_party/mujoco-3.6.0/model/adhesion/README.md)：MuJoCo adhesion 示例模型说明
- [third_party/mujoco-3.6.0/model/cube/README.md](third_party/mujoco-3.6.0/model/cube/README.md)：MuJoCo cube 示例模型说明
- [third_party/mujoco-3.6.0/model/humanoid/README.md](third_party/mujoco-3.6.0/model/humanoid/README.md)：MuJoCo humanoid 示例模型说明
- [third_party/mujoco-3.6.0/model/replicate/README.md](third_party/mujoco-3.6.0/model/replicate/README.md)：MuJoCo replicate 示例模型说明
- [third_party/mujoco-3.6.0/model/plugin/sdf/asset/README.md](third_party/mujoco-3.6.0/model/plugin/sdf/asset/README.md)：MuJoCo SDF 插件资产说明
- [reference/vision_msgs-humble/README.md](reference/vision_msgs-humble/README.md)：vision_msgs 参考资料说明
- [reference/vision_msgs-humble/vision_msgs_rviz_plugins/README.md](reference/vision_msgs-humble/vision_msgs_rviz_plugins/README.md)：RViz 插件英文说明
- [reference/vision_msgs-humble/vision_msgs_rviz_plugins/README.zh-CN.md](reference/vision_msgs-humble/vision_msgs_rviz_plugins/README.zh-CN.md)：RViz 插件中文说明

## 6. TODO（待更新）

- [ ] TODO: 障碍物生成器ros话题发布
- [ ] TODO: B2/G1机器人与对应链路
- [ ] TODO: 仿真器环境

## Designed by zdyukino
