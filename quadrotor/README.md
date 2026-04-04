# Quadrotor C++ Port

这个目录是四旋翼 MuJoCo 工程的 C++ 仿真与控制实现。

## 目录

当前仓库采用“业务代码与第三方依赖分离”的结构：

```text
ausim2/
├── CMakeLists.txt
├── assets/
│   └── crazyfile/
├── third_party/
│   └── mujoco_simulate/
│       ├── simulate.cc
│       ├── glfw_adapter.cc
│       ├── ...
│       ├── lodepng.h
│       ├── liblodepng.a
│       └── grabfiles.sh
└── quadrotor/
    ├── CMakeLists.txt
    ├── README.md
    ├── config.yaml
    ├── include/
    └── src/
```

- `quadrotor/`: 控制器、仿真逻辑、配置解析。
- `assets/`: 机型与场景资源。
- `third_party/`: 公用第三方源码与静态库，不再绑定在 `quadrotor/` 子目录下。

## 依赖

### 运行/构建硬依赖

- CMake >= 3.20
- C++17 编译器
- MuJoCo 3.3.x
- Eigen3
- yaml-cpp
- GLFW
- `third_party/mujoco_simulate` 下的 MuJoCo simulate 源文件与 `liblodepng.a`

Ubuntu 常用安装（除 MuJoCo 外）：

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev libglfw3-dev
```

MuJoCo CMake 查找优先使用：

- `mujoco_DIR`
- 环境变量 `MUJOCO_DIR`
- 环境变量 `MUJOCO_ROOT`

例如：

```bash
cmake -S . -B build -Dmujoco_DIR=/path/to/mujoco/build
```

## 抓取 MuJoCo simulate 依赖

`third_party/mujoco_simulate/` 依赖 MuJoCo 源码树中的 simulate 文件与 lodepng 产物。

在仓库根目录执行：

```bash
./third_party/mujoco_simulate/grabfiles.sh /path/to/mujoco-3.3.x
```

说明：

- 该脚本会从 MuJoCo 源码复制 `simulate/*.cc|*.h`。
- 该脚本会尝试复制 `lodepng.h` 与 `liblodepng.a`。
- 如果 `liblodepng.a` 未找到，通常需要先在 MuJoCo 源码目录完成一次 configure/build，再重新运行脚本。

## 构建

在仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
cmake --install build
```

主要 CMake target：

1. `quadrotor_mujoco_viewer`
   由 `third_party/mujoco_simulate/*.cc` 构建，封装官方 simulate viewer 相关逻辑。
2. `quadrotor_core`
   包含 `se3_controller.cpp`、`motor_mixer.cpp`、`geometry.cpp`、`quadrotor_sim.cpp`。
3. `quadrotor`
   主程序入口，链接 `quadrotor_core`。

## 运行

```bash
./build/bin/quadrotor [config.yaml] [--viewer|--headless]
```

配置文件行为：

- 若显式传入路径，则直接使用该路径。
- 若不传路径，只会尝试当前工作目录下的 `config.yaml`。
- 当前目录无 `config.yaml` 时会直接报错。

示例：

```bash
./build/bin/quadrotor ./quadrotor/config.yaml
./build/bin/quadrotor --viewer
./build/bin/quadrotor --headless
```

## 当前仿真与控制工作流

下面按“从 build target 到控制输出”的顺序说明。

### 1) 入口层：`quadrotor` 可执行程序

`main.cpp` 负责：

- 解析命令行参数（配置路径、viewer/headless 覆盖）。
- 调用 `LoadConfigFromYaml` 读取配置。
- 构造 `QuadrotorSim` 并执行 `Run()`。

### 2) 中间层：`QuadrotorSim` 仿真编排

`QuadrotorSim` 在运行时完成以下任务：

1. 加载 MuJoCo 模型与数据。
2. 解析并缓存 actuator id（`motor1`~`motor4`）。
3. 注册 `mjcb_control` 回调到 `QuadrotorSim::ControlCallback`。
4. 在 headless 或 viewer 模式下循环步进 `mj_step`。

### 3) 控制回调层：每个仿真步执行一次控制

每次回调进入 `ApplyControl`：

1. 读取当前状态 `State`（位置、速度、四元数、角速度）。
2. 调用 `BuildGoalState` 生成目标状态：
   - `use_circular_trajectory=false`：定点悬停。
   - `use_circular_trajectory=true`：先等待，再按圆轨迹给目标位置与前向方向。
3. 调用 `SE3Controller::controlUpdate` 计算控制命令。

### 4) 控制器层：`SE3Controller`

`SE3Controller` 的输入是：

- 当前状态 `current_state`
- 目标状态 `goal_state`
- 时间步长 `dt`（当前实现保留参数，未参与计算）
- 期望前向向量 `forward`

输出是 `ControlCommand`：

- `thrust`（标量）
- `angular`（三轴角控制量）

核心过程：

1. 平移误差计算：
   - 位置误差 `e_x = x - x_d`
   - 速度误差 `e_v = v - v_d`
2. 组合平移控制量 `trans_control`（包含 `kx`、`kv` 与目标速度项）。
3. 根据 `trans_control` 和 `forward` 构造期望姿态矩阵 `R_goal`。
4. 用几何误差（`veeMap(...)`）计算姿态误差与角速度误差。
5. 生成 `thrust` 与 `angular` 命令。

### 5) 执行层：混控与电机输入

`ApplyControl` 接到 `ControlCommand` 后：

1. 将 `thrust` 转为总推力：
   - `mixer_thrust = command.thrust * gravity * mass`
2. 将角控制量按 `torque_scale` 转为扭矩请求。
3. 传入 `MotorMixer::calculate(...)`，解算四个电机转速（krpm）。
4. 每个电机经过 `CalcMotorInput`：
   - 限幅转速
   - 用 `Ct * krpm^2` 计算推力
   - 归一化到 `[0,1]` 写入 `data->ctrl[...]`

这一步完成从“高层 SE3 控制命令”到“底层执行器输入”的闭环落地。

## 模型组织建议

建议保持下面分层：

- `quadrotor/`: 控制与仿真逻辑
- `assets/<airframe_name>/`: 机型 XML、网格、材质
- `third_party/`: 公用第三方依赖

这样可以稳定支持多机型切换，并避免第三方代码散落在业务目录中。
