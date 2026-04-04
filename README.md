# Quadrotor SE(3) Control with MuJoCo

这个工程演示了一个四旋翼在 MuJoCo 中的 SE(3) 控制闭环：

- `MuJoCo` 负责动力学仿真、传感器生成和执行器作用。
- `SE3Controller` 负责根据当前位置/姿态与目标状态计算总推力和机体扭矩控制量。
- `Mixer` 负责把总推力和三轴扭矩分配到 4 个电机。
- `main.py` 负责把以上几部分接起来，并启动 MuJoCo 可视化仿真。

仓库中的模型是基于 Crazyflie 2 的 MJCF 描述，仿真入口是根目录下的 `main.py`。

## 1. 工程结构

```text
Quadrotor_SE3_Control/
├── main.py
├── se3_controller.py
├── motor_mixer.py
├── geometry.py
├── calc_motor_speed.py
├── crazyfile/
│   ├── scene.xml
│   ├── cf2.xml
│   ├── README.md
│   └── assets/...
└── reference/
    └── 2010_Lee_tracking_quadrotor_se3.pdf
```

## 2. 所有 `.py` 文件的作用

### `main.py`

整个工程的主入口，也是 MuJoCo 仿真和控制闭环的连接点。

主要职责：

- 定义四旋翼参数，例如质量、重力、电机推力系数 `Ct`、反扭系数 `Cd`、力臂长度等。
- 提供电机相关换算函数：
  - 电机转速 -> 推力
  - 推力 -> 电机转速
  - 电机转速 -> MuJoCo 执行器输入（归一化到 `0~1`）
- 通过 `load_callback()` 加载 MuJoCo 模型：
  - 从 `./crazyfile/scene.xml` 创建 `MjModel`
  - 创建 `MjData`
  - 注册 `control_callback()` 为 MuJoCo 控制回调
- 在 `control_callback()` 中完成每一拍控制：
  - 从 `d.qpos / d.qvel / d.sensordata` 读取当前状态
  - 构造目标状态 `goal_state`
  - 调用 `SE3Controller.control_update()`
  - 调用 `Mixer.calculate()` 得到 4 个电机转速
  - 把电机转速映射成 MuJoCo actuator 的 `ctrl`
- 在 `if __name__ == '__main__':` 中通过 `viewer.launch(loader=load_callback)` 启动仿真窗口

你可以把它理解成：

`main.py = 仿真入口 + 状态读取 + 控制器调用 + 电机输出`

### `se3_controller.py`

这是核心控制器实现，负责 SE(3) 控制。

包含的主要类：

- `State`
  - 封装无人机状态：位置 `position`、速度 `velocity`、四元数 `quaternion`、角速度 `omega`
- `Control_Command`
  - 封装控制器输出：总推力 `thrust` 和三轴角控制量 `angular`
- `SE3Controller`
  - 控制器主体
  - `update_linear_error()` 计算位置和速度误差
  - `update_angular_error()` 根据平移控制量和期望机头方向 `forward` 构造目标姿态，并计算姿态误差
  - `control_update()` 输出最终的推力和角控制量

控制逻辑可以概括为：

1. 根据位置误差和速度误差计算平移控制量 `trans_control`
2. 根据 `trans_control` 推出机体期望 `z` 轴方向
3. 结合期望机头方向 `forward` 构造目标旋转矩阵 `R_goal`
4. 用 SO(3) 误差计算姿态误差 `e_R`
5. 输出总推力和三轴角速度/扭矩控制量

### `motor_mixer.py`

这是四旋翼电机混控器，把机体控制量分配到四个电机。

主要职责：

- 定义四旋翼混控矩阵 `mat`
- 计算其逆矩阵 `inv_mat`
- 根据输入：
  - 总推力 `thrust`
  - 机体系扭矩 `mx, my, mz`
- 反解出四个电机的转速平方，再开方得到电机转速

这个文件还有一个很重要的特性：它做了饱和处理。

- 先优先分配 `X/Y` 轴扭矩
- 如果出现电机超限或负值，会对控制量缩放
- 在 `X/Y` 轴还有余量时，再尝试分配 `Z` 轴扭矩

也就是说，这个 `Mixer` 不是简单求逆，而是带“限幅和取舍策略”的混控器。

### `geometry.py`

这是控制器依赖的几何工具库，主要处理四元数、旋转矩阵和 Lie 群/李代数相关运算。

主要内容：

- `GeoQuaternion`
  - 四元数表示
  - 四元数归一化
  - 角轴 -> 四元数
  - 旋转矩阵 -> 四元数
  - 四元数 -> 旋转矩阵
  - 球面线性插值 `slerp`
- `veemap()`
  - 把反对称矩阵映射为向量
- `hatmap()` / `skewSym()`
  - 把三维向量映射为反对称矩阵
- `so3LieToMat()` / `se3LieToRotTrans3()` / `se3LieToMat4()`
  - Lie algebra 到旋转/位姿矩阵的转换
- `rotTrans3ToMat4()`
  - 旋转矩阵和平移向量转齐次矩阵

在当前工程里，`se3_controller.py` 主要使用了：

- `GeoQuaternion.getRotationMatrix()`
- `veemap()`

### `calc_motor_speed.py`

这是一个独立的小测试脚本，用 `scipy.optimize.fsolve` 求解四个电机转速。

特点：

- 它不会被主程序调用
- 更像是混控公式推导或调试时的验证脚本
- 文件开头也标注了 `Test Failed`

因此可以把它看成：

`calc_motor_speed.py = 电机分配求解的实验/测试脚本`

## 3. MuJoCo 仿真是如何被调用起来的

这是这个工程最关键的一部分。

### 3.1 启动入口

运行：

```bash
python main.py
```

程序会执行：

```python
viewer.launch(loader=load_callback)
```

这里使用的是 MuJoCo 官方 Python 接口里的可视化 viewer。

### 3.2 `load_callback()` 做了什么

`load_callback()` 是传给 `viewer.launch()` 的加载函数，它会：

1. 先清除已有控制回调：

```python
mujoco.set_mjcb_control(None)
```

2. 从 XML 加载模型：

```python
m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
```

3. 创建仿真数据对象：

```python
d = mujoco.MjData(m)
```

4. 注册控制回调：

```python
mujoco.set_mjcb_control(lambda m, d: control_callback(m, d))
```

MuJoCo 后续每一步仿真推进时，都会调用这个 `control_callback()`。

### 3.3 `scene.xml` 和 `cf2.xml` 的关系

- `crazyfile/scene.xml`
  - 是场景级 XML
  - 定义地面、灯光、相机、天空盒
  - 通过 `<include file="cf2.xml"/>` 引入无人机本体

- `crazyfile/cf2.xml`
  - 是四旋翼本体模型
  - 定义刚体质量和惯量
  - 定义 `freejoint`，使无人机可以在空间中自由运动
  - 定义 4 个电机 actuator
  - 定义 IMU 传感器：
    - `gyro`
    - `accelerometer`
    - `framequat`

所以调用链是：

`main.py -> scene.xml -> cf2.xml -> actuator/sensor/body`

### 3.4 MuJoCo 每一步里发生了什么

每次 MuJoCo 调用 `control_callback(m, d)` 时，主程序会按下面流程运行：

1. 从 `d` 中读取当前状态

- `d.qpos`：位置和自由关节姿态
- `d.qvel`：线速度和角速度
- `d.qacc`：加速度
- `d.sensordata`：传感器输出

当前代码里实际主要使用：

- `gyro` 作为当前角速度
- `framequat` 作为当前姿态四元数
- `qpos` 作为当前位置
- `qvel` 作为当前速度

2. 构造目标状态

当前默认是定点悬停：

```python
goal_pos = np.array([0.0, 0.0, 0.3])
goal_heading = np.array([1.0, 0.0, 0.0])
```

也可以切换成圆轨迹：

```python
goal_pos, goal_heading = simple_trajectory(d.time)
```

3. 调用 SE(3) 控制器

```python
control_command = ctrl.control_update(curr_state, goal_state, dt, forward)
```

输出包括：

- `ctrl_thrust`：总推力控制量
- `ctrl_torque`：三轴角控制量

4. 调用混控器把机体控制量分配到四个电机

```python
motor_speed = mixer.calculate(mixer_thrust, mixer_torque[0], mixer_torque[1], mixer_torque[2])
```

5. 把电机转速转成 MuJoCo actuator 输入

```python
d.actuator('motor1').ctrl[0] = calc_motor_input(motor_speed[0])
...
```

这里 `calc_motor_input()` 会把电机转速映射到 `0~1`，对应 `cf2.xml` 中电机 actuator 的 `ctrlrange="0 1"`。

### 3.5 MuJoCo 中四个电机 actuator 是如何工作的

在 `crazyfile/cf2.xml` 中，4 个电机是通过 `<motor>` 定义的，例如：

```xml
<motor ctrlrange="0 1" gear="0 0 0.1573 0 0 -3.842e-03" site="motor1_site" name="motor1"/>
```

含义可以这样理解：

- `ctrlrange="0 1"`
  - 控制输入是归一化值
- `site="motor1_site"`
  - 力和力矩施加在对应电机位置
- `gear="0 0 0.1573 0 0 -3.842e-03"`
  - 输入为 1 时，沿机体 `z` 方向施加最大推力 `0.1573 N`
  - 同时施加对应旋翼反扭矩

四个电机中，`Mz` 的符号交替相反，用来表示顺/逆时针桨带来的反扭矩方向差异。

## 4. 整个工程怎么使用

### 4.1 安装依赖

这个工程直接依赖的 Python 包主要有：

- `mujoco`
- `numpy`
- `scipy`（仅 `calc_motor_speed.py` 用到）

建议使用 Python 3.10+，安装方式例如：

```bash
pip install mujoco numpy scipy
```

如果你的系统没有图形界面，`viewer.launch()` 无法正常弹出窗口。

### 4.2 运行仿真

在仓库根目录执行：

```bash
python main.py
```

正常情况下会打开 MuJoCo viewer，并看到四旋翼在场景中起飞并尝试稳定在目标点附近。

### 4.3 修改目标轨迹

当前 `main.py` 里默认使用的是定点悬停目标：

```python
goal_pos, goal_heading = np.array([0.0, 0.0, 0.3]), np.array([1.0, 0.0, 0.0])
```

如果想改成圆轨迹，把下面这行取消注释：

```python
goal_pos, goal_heading = simple_trajectory(d.time)
```

并注释掉定点目标那一行即可。

### 4.4 调控制参数

`main.py` 中控制器参数初始化如下：

```python
ctrl.kx = 0.6
ctrl.kv = 0.4
ctrl.kR = 6.0
ctrl.kw = 1.0
```

含义大致是：

- `kx`：位置误差反馈增益
- `kv`：速度误差反馈增益
- `kR`：姿态误差反馈增益
- `kw`：角速度误差反馈增益

如果出现：

- 起飞慢、悬停下沉：优先检查推力缩放和位置增益
- 姿态晃动大：优先减小 `kR` / `kw` 或重新调 `torque_scale`
- 偏航控制弱：检查 `Mixer` 和 `cf2.xml` 中 `Mz` 的符号与尺度

### 4.5 调扭矩缩放

`main.py` 中还有一个关键参数：

```python
torque_scale = 0.001
```

它把控制器输出的无量纲角控制量缩放为实际扭矩 `Nm`。  
如果这个值太大，飞机会抖动或快速翻转；太小则姿态响应会很弱。

## 5. 代码执行流程总览

完整链路如下：

1. `python main.py`
2. `viewer.launch(loader=load_callback)`
3. `load_callback()` 加载 `scene.xml`
4. `scene.xml` include `cf2.xml`
5. MuJoCo 创建 `MjModel` 和 `MjData`
6. MuJoCo 每个仿真步调用 `control_callback(m, d)`
7. `control_callback()` 读取传感器和状态
8. `SE3Controller` 输出总推力和姿态控制量
9. `Mixer` 分配到四个电机转速
10. `main.py` 把电机转速写入 `d.actuator(...).ctrl[0]`
11. MuJoCo 根据 actuator、刚体、接触和积分器推进下一步仿真

## 6. 一些实现细节说明

### 状态来源

当前代码混合使用了两类状态来源：

- `d.qpos / d.qvel`：直接从仿真状态读取位置和速度
- `d.sensordata`：读取 IMU 和姿态四元数

这符合“仿真真值 + 传感器量”的演示写法，但如果后续要更贴近真实飞控，通常会进一步加入状态估计器。

### 单位约定

- 位置：`m`
- 速度：`m/s`
- 角速度：`rad/s`
- 推力：`N`
- 扭矩：`Nm`
- 电机转速：代码里使用 `krpm`

### 参考论文

`reference/2010_Lee_tracking_quadrotor_se3.pdf`  
这个文件大概率就是 SE(3) 控制律的理论参考来源。

## 7. 快速修改入口

如果你准备继续开发，最常改的地方通常是：

- 改仿真入口和目标轨迹：`main.py`
- 改控制器：`se3_controller.py`
- 改电机分配：`motor_mixer.py`
- 改模型/执行器/传感器：`crazyfile/cf2.xml`
- 改场景：`crazyfile/scene.xml`

## 8. 当前仓库的一点补充说明

- `calc_motor_speed.py` 是测试脚本，不参与主流程
- `geometry.py` 中有少量注释编码显示异常，但不影响代码运行
- 本地环境里 `numpy` 与 `scipy` 版本组合会给出兼容性警告；主仿真依赖 `main.py`、`mujoco`、`numpy`，而 `scipy` 只影响测试脚本

## 9. 最短使用步骤

如果你只是想先跑起来：

```bash
pip install mujoco numpy scipy
python main.py
```

如果你只想看 MuJoCo 仿真主链路，请优先读这几个文件：

1. `main.py`
2. `se3_controller.py`
3. `motor_mixer.py`
4. `crazyfile/cf2.xml`
5. `crazyfile/scene.xml`

## 10. 如果改成 C++ MuJoCo，大概会怎么重构

如果后续你想把这个项目从 Python 版迁到 C++ 版 MuJoCo，最自然的思路不是“逐行翻译”，而是把现在的职责边界保留下来，再换成 C++ 的工程组织方式。

可以先记住一个总原则：

- `模型文件 XML` 基本不用改
- `控制逻辑` 可以几乎原样迁移
- `MuJoCo 调用方式` 会从 Python API 改为 C API
- `工程结构` 会从脚本式入口改成模块化 C++ 工程

### 10.1 哪些部分可以直接保留

下面这些内容基本都可以继续沿用：

- `crazyfile/scene.xml`
- `crazyfile/cf2.xml`
- `assets/` 下的网格模型
- 控制器中的数学逻辑
- mixer 的动力分配公式
- 几何工具中的四元数、旋转矩阵、Lie algebra 相关函数

也就是说，迁移到 C++ 时，最值得保留的是“模型”和“算法”，主要重写的是“程序组织形式”和“接口层”。

### 10.2 建议的 C++ 工程结构

一个比较适合这个项目的重构方式如下：

```text
Quadrotor_SE3_Control/
├── CMakeLists.txt
├── README.md
├── assets/
├── models/
│   ├── scene.xml
│   └── cf2.xml
├── include/
│   ├── controller/state.hpp
│   ├── controller/se3_controller.hpp
│   ├── control/motor_mixer.hpp
│   ├── math/geometry.hpp
│   └── sim/quadrotor_sim.hpp
├── src/
│   ├── main.cpp
│   ├── se3_controller.cpp
│   ├── motor_mixer.cpp
│   ├── geometry.cpp
│   └── quadrotor_sim.cpp
└── third_party/  (可选)
```

对应关系大致是：

- `main.py` -> `src/main.cpp`
- `se3_controller.py` -> `src/se3_controller.cpp`
- `motor_mixer.py` -> `src/motor_mixer.cpp`
- `geometry.py` -> `src/geometry.cpp`
- MuJoCo 的加载/仿真循环逻辑 -> `src/quadrotor_sim.cpp`

### 10.3 Python 版到 C++ 版的模块映射

建议把当前 `main.py` 拆成两个部分：

1. `main.cpp`
   - 负责程序启动
   - 解析配置
   - 创建仿真器对象
   - 启动 viewer 或主循环

2. `quadrotor_sim.cpp`
   - 负责加载 MuJoCo 模型
   - 创建 `mjModel` 和 `mjData`
   - 实现 control callback
   - 管理仿真步进
   - 对外暴露 `step()`、`reset()`、`run()` 等接口

这样会比把所有逻辑都塞进 `main.cpp` 清晰很多。

### 10.4 MuJoCo 调用方式会怎么变化

Python 版里现在是：

```python
m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
d = mujoco.MjData(m)
mujoco.set_mjcb_control(lambda m, d: control_callback(m, d))
viewer.launch(loader=load_callback)
```

迁到 C++ 后，核心调用会变成更接近下面这种形式：

```cpp
char error[1000] = "Could not load XML";
mjModel* model = mj_loadXML("assets/crazyfile/scene.xml", nullptr, error, sizeof(error));
mjData* data = mj_makeData(model);
mjcb_control = controlCallback;
```

然后在主循环里做：

```cpp
while (!done) {
    mj_step(model, data);
}
```

如果要接 viewer，还会额外接入 MuJoCo 的可视化和窗口循环。

### 10.5 控制回调在 C++ 里会怎么写

当前 Python 逻辑是：

1. 从 `d.qpos / d.qvel / d.sensordata` 读状态
2. 构造 `curr_state` 和 `goal_state`
3. 调 `SE3Controller`
4. 调 `Mixer`
5. 写回 `d.actuator(...).ctrl[0]`

迁移到 C++ 后，建议保留同样顺序：

```cpp
void controlCallback(const mjModel* m, mjData* d) {
    State curr = readCurrentState(m, d);
    State goal = buildGoalState(d->time);

    ControlCommand cmd = controller.controlUpdate(curr, goal, dt, forward);
    MotorCommand motor = mixer.calculate(total_thrust, mx, my, mz);

    d->ctrl[motor1_id] = toActuatorInput(motor.w1);
    d->ctrl[motor2_id] = toActuatorInput(motor.w2);
    d->ctrl[motor3_id] = toActuatorInput(motor.w3);
    d->ctrl[motor4_id] = toActuatorInput(motor.w4);
}
```

和 Python 的最大区别在于：

- Python 用名字 `d.actuator('motor1')`
- C++ 里更常见的做法是先查 actuator id，再写 `d->ctrl[id]`

例如：

```cpp
int motor1_id = mj_name2id(model, mjOBJ_ACTUATOR, "motor1");
```

### 10.6 数据结构建议怎么改

当前 Python 里有：

- `State`
- `Control_Command`
- `Mixer`
- `SE3Controller`

在 C++ 里建议明确写成结构体和类：

```cpp
struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d omega;
};

struct ControlCommand {
    double thrust;
    Eigen::Vector3d angular;
};
```

然后：

- `SE3Controller` 保持为类
- `Mixer` 保持为类
- `geometry.py` 中的功能迁到 `namespace geometry`

这里非常建议直接引入 `Eigen`，因为：

- 旋转矩阵和四元数会更好写
- 矩阵运算更自然
- 可以减少自己维护底层数组的负担

### 10.7 几何模块怎么迁

`geometry.py` 迁到 C++ 时，不建议完全手写一套数学容器，通常有两种选择：

1. 推荐方案：以 `Eigen` 为主
   - 四元数用 `Eigen::Quaterniond`
   - 向量用 `Eigen::Vector3d`
   - 矩阵用 `Eigen::Matrix3d`
   - 自己只保留 `vee`、`hat`、`so3 exp` 等少量工具函数

2. 完全自写
   - 可以做，但工作量更大
   - 后续维护成本也更高

对这个项目来说，第一种方案通常更合适。

### 10.8 mixer 模块怎么迁

`motor_mixer.py` 在 C++ 里几乎可以直接照着重写。

因为它本质上就是：

- 若干常量
- 一个 4x4 混控矩阵
- 逆矩阵求解
- 饱和裁剪逻辑

如果用了 `Eigen`，写起来会很直接：

```cpp
Eigen::Matrix4d mat;
Eigen::Matrix4d inv_mat;
Eigen::Vector4d control_input;
Eigen::Vector4d motor_speed_sq;
```

这部分是迁移里最简单的一块。

### 10.9 目标轨迹怎么迁

当前 `simple_trajectory(time)` 是一个独立函数。  
迁到 C++ 后，建议不要继续写死在控制回调里，而是抽象成单独模块，例如：

- `trajectory.hpp`
- `trajectory.cpp`

对外提供：

```cpp
GoalSample sampleCircleTrajectory(double time);
GoalSample sampleHoverTrajectory(double time);
```

这样以后你想增加：

- 8 字轨迹
- 螺旋轨迹
- waypoint 路径

都会更方便。

### 10.10 viewer 和线程模型需要单独设计

Python 版现在直接：

```python
viewer.launch(loader=load_callback)
```

这对于演示很方便，但如果换成 C++，通常需要更明确地区分：

- 仿真线程
- 渲染线程
- 控制回调
- UI 事件循环

如果只是想先复现当前功能，建议第一阶段先不要做复杂界面，优先做：

1. 命令行启动
2. 加载 XML
3. 控制回调
4. `mj_step`
5. 基本 viewer

先把飞行闭环跑通，再考虑更复杂的交互。

### 10.11 配置项建议外置

现在很多参数直接写在 `main.py`：

- 质量
- 重力
- `Ct`
- `Cd`
- 控制增益
- `torque_scale`

如果迁到 C++，建议把这部分单独抽成配置文件，例如：

- `config/controller.yaml`
- `config/vehicle.yaml`

这样好处很明显：

- 调参不需要重新编译
- 不同机型可以共用同一套程序
- 更适合后续做批量实验

### 10.12 建议的重构阶段

如果你真的要迁移，推荐按下面顺序做，而不是一次性全部重写。

第一阶段：先做最小可运行版本

- 用 C++ 成功加载 `scene.xml`
- 完成 `mjModel` / `mjData` 初始化
- 实现一个固定推力或悬停控制
- 确认 actuator 和 sensor 索引都正确

第二阶段：迁移控制器

- 迁移 `State`
- 迁移 `SE3Controller`
- 迁移 `geometry`
- 迁移 `Mixer`

第三阶段：做结构优化

- 加入配置文件
- 把轨迹生成独立成模块
- 加入日志系统
- 加入更清晰的类封装

第四阶段：做工程化增强

- 单元测试
- 参数热加载
- 数据记录和回放
- 与 ROS2 / MPC / estimator 对接

### 10.13 对这个项目最实际的迁移结论

如果你的目标是：

- 验证控制算法
- 调整增益
- 跑演示仿真

那继续保留 Python 版通常是性价比最高的。

如果你的目标变成：

- 做高频实时控制
- 做大规模仿真集成
- 接入现有 C++ 飞控或机器人软件栈
- 做更严肃的工程部署

那就值得迁到 C++。

一句话总结这个项目的迁移方式：

`保留 XML 和控制数学，重写仿真接口层与工程结构，把 Python 脚本式主程序改造成 C++ 模块化仿真框架。`
