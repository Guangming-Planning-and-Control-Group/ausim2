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

### `quadrotor/include/` 头文件职责

- `include/controller/state.hpp`
  定义控制与仿真共用的数据结构，例如 `State` 和 `ControlCommand`。把这些基础类型单独抽出来，可以避免控制器、仿真器、混控器互相重复定义同一套状态接口。
- `include/controller/se3_controller.hpp`
  声明 SE(3) 几何控制器，包括增益、当前/目标状态缓存，以及 `controlUpdate(...)` 主接口。它负责“从状态误差到期望总推力与角控制量”的控制律表达。
- `include/control/motor_mixer.hpp`
  声明电机混控器 `MotorMixer` 与 `MixerParams`。它负责把总推力和三轴力矩请求分解成四个电机的转速，是“高层控制输出”到“执行器分配”之间的桥接层。
- `include/math/geometry.hpp`
  声明姿态/李群相关的几何工具函数，例如四元数转旋转矩阵、`hat/vee` 映射、`se3` 变换构造等。它只放纯数学工具，避免控制器代码里混入大量底层矩阵操作细节。
- `include/sim/quadrotor_sim.hpp`
  声明仿真编排类 `QuadrotorSim` 和配置结构体。它负责加载模型、读取 YAML、接入 MuJoCo callback、组织 viewer/headless 运行流程，是整个程序的运行时外壳。

### 头文件按子目录拆分的理由

- `controller/`
  放“控制语义”相关接口，即状态定义和控制器本体。这里关注的是控制问题本身，而不是 MuJoCo 或执行器实现细节。
- `control/`
  放控制分配与执行器映射逻辑。`MotorMixer` 虽然服务于控制链，但职责更接近 actuator allocation，而不是姿态/轨迹控制律，所以和 `controller/` 分开更清晰。
- `math/`
  放可复用的底层几何工具。这样控制器和未来其他模块都能共享同一套数学原语，减少重复实现，也便于单独替换或扩展。
- `sim/`
  放仿真驱动、配置、MuJoCo 绑定和运行调度。把它和控制器分开，可以让控制算法保持相对独立，后续若切换仿真后端或加入实机接口时，边界会更稳定。

这套划分本质上是在区分“数学工具”“控制算法”“执行器分配”“仿真集成”四层职责，减少头文件相互缠绕，也让新文件更容易找到合适归属。

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

viewer 配置补充：

- `viewer.mjui_enabled=false` 时，会隐藏官方 MuJoCo simulate viewer 的左右侧 UI 面板，以及 help/info/profiler/sensor overlay，保留纯 3D 视图。

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

### 3) 控制回调层：每个仿真步调用，但控制可按降采样频率更新

每次回调进入 `ApplyControl`：

1. 读取当前状态 `State`（位置、速度、四元数、角速度）。
2. 调用 `BuildGoalState` 生成目标状态：
   - `example_mode=1`：简单示例，位置控制时用于悬停；速度控制时在 `wait_time` 前先保持到 `goal.position`，之后跟踪 `goal.velocity`。
   - `example_mode=2`：圆轨迹示例。位置控制时给位置圆轨迹；速度控制时在 `wait_time` 前先保持到 `goal.position`，之后将 `goal.velocity` 周期旋转，并同步更新 `heading` 以飞圆。
3. 仅当达到控制更新周期时，调用 `SE3Controller::controlUpdate` 计算新控制命令。
4. 非控制更新步复用上一次控制命令，直到下一个控制周期到来。

### 4) 控制器层：`SE3Controller`

`SE3Controller` 的输入是：

- 当前状态 `current_state`
- 目标状态 `goal_state`
- 控制步长 `dt`（当前等于 `control_decimation * simulation.dt`，实现中暂未参与控制律计算）
- 期望前向向量 `forward`
- 控制模式 `control_mode`
  - `0`：预留给直接推力/电机控制，当前未实现
  - `1`：速度控制，只使用速度误差项
  - `2`：位置控制，同时使用位置误差和速度误差

输出是 `ControlCommand`：

- `thrust`（标量）
- `angular`（三轴角控制量）

核心过程：

1. 平移误差计算：
   - 位置误差 `e_x = x - x_d`
   - 速度误差 `e_v = v - v_d`
2. 组合平移控制量 `trans_control`：
   - 位置控制模式下使用 `kx * e_x + kv * e_v`
   - 速度控制模式下忽略位置误差，只使用速度误差项
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

## 频率配置

当前实现将“物理频率”和“控制频率”解耦：

- `simulation.dt` 决定 MuJoCo 物理积分步长，例如 `0.001` 对应 `1000 Hz`。
- `controller.rate_hz` 决定控制命令更新频率，例如 `250.0` 表示每 4 个物理步更新一次控制。

## 控制模式与示例模式

`simulation` 下新增两个模式字段：

- `control_mode`
  - `0`：预留给纯转速/推力控制，当前未实现，若选中会直接报错
  - `1`：速度控制
  - `2`：位置控制
- `example_mode`
  - `1`：简单示例
  - `2`：圆轨迹示例

推荐组合：

- `control_mode=2`，`example_mode=1`
  位置控制悬停到 `goal.position`
  此时会忽略 `goal.velocity`，按零目标速度处理
- `control_mode=1`，`example_mode=1`
  `wait_time` 前先位置保持到 `goal.position`
  `wait_time` 后切换到速度控制并跟踪 `goal.velocity`
  当 `goal.velocity=[0, 0, 0]` 时，切换后表现为零速悬停
- `control_mode=2`，`example_mode=2`
  位置控制圆轨迹
- `control_mode=1`，`example_mode=2`
  `wait_time` 前先位置保持到 `goal.position`
  `wait_time` 后切换到速度控制，将 `goal.velocity` 在水平面内按 `speed_hz` 周期旋转，并同步更新 `heading`，形成飞圆 demo

相关配置项：

```yaml
simulation:
  control_mode: 1
  example_mode: 2

goal:
  position: [0.0, 0.0, 0.3]
  velocity: [0.0, 0.0, 0.0]
  heading: [1.0, 0.0, 0.0]

trajectory:
  wait_time: 1.5
  height: 0.3
  radius: 0.5
  speed_hz: 0.3
  height_gain: 1.5
```

默认配置即：

```yaml
simulation:
  dt: 0.001

controller:
  rate_hz: 250.0
```

说明：

- 当前实现要求 `1 / simulation.dt / controller.rate_hz` 为整数，也就是控制频率必须能整除物理频率。
- 如果不满足，程序会在启动时直接报错，避免出现隐式抖动或不稳定的采样节拍。
- 等效控制步长为 `1 / controller.rate_hz`，也等于 `control_decimation * simulation.dt`。

常见配置示例：

- `simulation.dt = 0.001`，`controller.rate_hz = 250.0`：物理 `1000 Hz`，控制每 4 个物理步更新一次。
- `simulation.dt = 0.0005`，`controller.rate_hz = 250.0`：物理 `2000 Hz`，控制每 8 个物理步更新一次。
- `simulation.dt = 0.001`，`controller.rate_hz = 333.0`：非法配置，因为 `1000 / 333` 不是整数。

## 模型组织建议

建议保持下面分层：

- `quadrotor/`: 控制与仿真逻辑
- `assets/<airframe_name>/`: 机型 XML、网格、材质
- `third_party/`: 公用第三方依赖

这样可以稳定支持多机型切换，并避免第三方代码散落在业务目录中。
