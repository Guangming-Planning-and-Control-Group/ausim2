# ausim2

## 1. 介绍仓库

`ausim2` 是一个基于 MuJoCo 3.6.0 的仿真仓库，当前提供两个主要可执行程序：

- `quadrotor`：四旋翼仿真，集成 ray-caster 深度传感器插件与动态障碍场景生成。
- `scout`：地面车辆仿真。

仓库当前的 MuJoCo 依赖已经完全内置，不再依赖外部 MuJoCo 源码或 `/opt/mujoco` 安装。核心依赖位于：

- `third_party/mujoco-3.6.0/include/mujoco`：MuJoCo 公开头文件
- `third_party/mujoco-3.6.0/lib/libmujoco.so.3.6.0`：MuJoCo 运行时库
- `third_party/mujoco-3.6.0/bin/mujoco_plugin/`：MuJoCo 官方插件与 OBJ/STL decoder
- `build/bin/mujoco_plugin/libsensor_raycaster.so`：本仓库编译出的自定义传感器插件

仓库中和运行直接相关的目录如下：

- `assets/`：MJCF 模型、网格与场景文件
- `quadrotor/`：四旋翼仿真源码与配置
- `ground_vehicle/`：Scout 仿真源码与配置
- `ausim_common/`：配置、IPC、ROS2 bridge 相关公共代码
- `third_party/`：MuJoCo release 包、ray-caster 插件源码、动态障碍生成器
- `em_run.sh`：统一启动脚本

## 2. 如何编译相关代码

系统依赖：

- CMake >= 3.20
- C++17 编译器
- Eigen3
- yaml-cpp
- GLFW
- ROS2 Humble

安装常用 apt 依赖并构建：

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev libglfw3-dev
source /opt/ros/humble/setup.bash
cmake -S . -B build
cmake --build build -j
```

主要产物：

- `build/bin/quadrotor`
- `build/bin/scout`
- `build/bin/quadrotor_ros_bridge`
- `build/bin/ausim_ros_bridge`
- `build/bin/mujoco_plugin/libsensor_raycaster.so`

可选地运行一次 MuJoCo 插件 smoke test：

```bash
./build/bin/mujoco_ray_caster_smoketest
```

## 3. 如何启动

推荐使用统一启动脚本：

```bash
./em_run.sh --headless
./em_run.sh --viewer
./em_run.sh -S --headless
```

`em_run.sh` 的行为：

- 自动在 `quadrotor` 和 `scout` 之间选择启动目标，并把默认选择保存到 `.em_run_target`
- 自动导出 `MUJOCO_PLUGIN_DIR`
- 运行 `quadrotor` 时，会先调用 `third_party/dynamic_obs_generator/generate_scene_obstacles.py`

运行时会同时使用两组插件目录：

- `third_party/mujoco-3.6.0/bin/mujoco_plugin`：MuJoCo 官方插件与 decoder
- `build/bin/mujoco_plugin`：本仓库编译出的 `libsensor_raycaster.so`

也可以直接运行可执行文件。

四旋翼：

```bash
./build/bin/quadrotor --sim-config ./quadrotor/cfg/sim_config.yaml --headless
./build/bin/quadrotor --sim-config ./quadrotor/cfg/sim_config.yaml --viewer
./build/bin/quadrotor --sim-config ./quadrotor/cfg/sim_config.yaml \
  --robot-config ./quadrotor/cfg/robot/xi35_config.yaml --headless
```

地面车辆：

```bash
./build/bin/scout --sim-config ./ground_vehicle/cfg/sim_config.yaml --headless
./build/bin/scout --sim-config ./ground_vehicle/cfg/sim_config.yaml --viewer
```

默认配置文件：

- `quadrotor/cfg/sim_config.yaml`，默认指向 `robot/crazyfile_config.yaml`
- `ground_vehicle/cfg/sim_config.yaml`，默认指向 `robot/scout_v2_config.yaml`

如果需要在运行时使用 ROS2 命令，请先在当前 shell 中执行：

```bash
source /opt/ros/humble/setup.bash
```

更细的源码结构与单模块说明见：

- [quadrotor/README.md](quadrotor/README.md)
- [third_party/README.md](third_party/README.md)

Designed by zdyukino