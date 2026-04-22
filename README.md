# ausim2

## 1. 介绍仓库

`ausim2` 是一个基于 MuJoCo 3.6.0 的仿真仓库，当前提供两个主要可执行程序：

- `quadrotor`：四旋翼仿真，集成 ray-caster 深度传感器插件与动态障碍场景生成。
- `ground_vehicle`：地面车辆仿真。

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
3. 在仓库根 `build/` 下执行 `cmake` 配置与并行编译

主要产物：

- `build/bin/quadrotor`
- `build/bin/scout`
- `build/bin/quadrotor_ros_bridge`
- `build/bin/ausim_ros_bridge`
- `build/bin/mujoco_plugin/libsensor_raycaster.so`

系统预置要求：ROS2 Humble（`/opt/ros/humble/setup.bash`）、`apt`（Ubuntu 22.04 系）；其他依赖由脚本负责。

可选的 MuJoCo 插件 smoke test：

```bash
./build/bin/mujoco_ray_caster_smoketest
```

## 3. 启动

推荐使用统一启动脚本 `./em_run.sh`：

```bash
./em_run.sh --headless     # 无 GUI
./em_run.sh --viewer       # 带 MuJoCo viewer
./em_run.sh -S --headless  # 重新选择仿真目标（quadrotor / scout）
```

`em_run.sh` 的行为：

- 首次运行提示选择仿真目标（`quadrotor` 或 `scout`），并保存到 `.em_run_target`
- 自动导出 `MUJOCO_PLUGIN_DIR`（同时覆盖仓内 vendored 目录与 `build/bin/mujoco_plugin`）
- 启动 `quadrotor` 时如对应 registry 开启动态障碍，会先调用 `third_party/dynamic_obs_generator/generate_scene_obstacles.py` 生成场景
- 支持透传 `--sim-config` / `--robot-config` / `--headless` / `--viewer` 等参数

默认配置文件：

- `quadrotor/cfg/sim_config.yaml` → `robot/crazyfile_config.yaml`
- `ground_vehicle/cfg/sim_config.yaml` → `robot/scout_v2_config.yaml`

如需在另一个 shell 中使用 ROS2 命令，请先：

```bash
source /opt/ros/humble/setup.bash
```

## 4. 模块文档

- [quadrotor/README.md](quadrotor/README.md)：四旋翼分层结构、配置、状态机（teleop）、传感器扩展
- [ground_vehicle/README.md](ground_vehicle/README.md)：Scout 分层结构、配置、状态机
- [third_party/README.md](third_party/README.md)：vendored / 外部组件说明

## Designed by zdyukino
