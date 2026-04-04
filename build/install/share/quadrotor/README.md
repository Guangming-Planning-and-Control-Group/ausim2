# Quadrotor C++ Port

这个目录是原 Python 四旋翼 MuJoCo 项目的 C++ 版骨架。

## 目录

```text
quadrotor/
├── CMakeLists.txt
├── README.md
├── config.yaml
├── include/
└── src/
```

机型资源不再放在 `quadrotor/` 里重复维护，统一使用仓库根目录的模型资源：

```text
assets/
└── crazyfile/
    ├── scene.xml
    ├── cf2.xml
    └── assets/...
```

`quadrotor/config.yaml` 默认直接引用：

```yaml
model:
  scene_xml: ../assets/crazyfile/scene.xml
```

这样后续接入不同型号四旋翼时，只需要新增一个新的资源目录，例如：

```text
assets/
├── crazyfile/
├── iris/
└── x500/
```

然后在配置文件里切换：

```yaml
model:
  scene_xml: ../assets/iris/scene.xml
```

## 依赖

### 当前硬依赖

当前这版 C++ 工程已经直接使用了下面这些依赖：

- `CMake >= 3.20`
- `C++17` 编译器
- `MuJoCo 3.3.7`
- `Eigen3`
- `yaml-cpp`
- `GLFW`

在 Ubuntu 上，除 MuJoCo 外通常可以先安装：

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev
```

MuJoCo 当前默认会优先尝试这些路径：

- `mujoco_DIR`
- 环境变量 `MUJOCO_DIR`
- 环境变量 `MUJOCO_ROOT`
- `/home/x/mujoco/mujoco-3.3.7/build`

如果你的 MuJoCo 不在这些位置，可以这样指定：

```bash
cmake -S . -B build -Dmujoco_DIR=/path/to/mujoco/build
```

## 可选依赖

当前工程已经支持 `headless` 和 `GLFW + MuJoCo viewer` 两种运行方式。  
但如果后面你准备继续把它做成更完整的 C++ 仿真工程，下面这些依赖会明显更省事。

### 1. `GLFW`

用途：

- 接 MuJoCo 的 C++ 可视化 viewer
- 做窗口、键鼠事件和 OpenGL 上下文管理

建议场景：

- 你想把现在的 headless 仿真改成带界面的 `viewer`
- 你想直接复用 MuJoCo 自带的 `simulate` 相关代码

Ubuntu 常见安装：

```bash
sudo apt install libglfw3-dev
```

### 2. `fmt` 或 `spdlog`

用途：

- 更舒服的日志输出
- 比 `std::cout` 更适合后面扩展调试信息、状态记录和分级日志

建议场景：

- 你准备频繁打印控制器状态、电机转速、误差量
- 后面想把日志写文件，或者区分 `info/warn/error`

推荐：

- 只想格式化输出：`fmt`
- 想直接上日志系统：`spdlog`

Ubuntu 常见安装：

```bash
sudo apt install libfmt-dev libspdlog-dev
```

### 3. `CLI11` 或 `cxxopts`

用途：

- 更方便地处理命令行参数
- 比当前手写的 `argv` 解析更适合后面增加运行模式

建议场景：

- 你想支持：
  - 指定配置文件
  - 切换 hover / trajectory
  - 指定仿真时长
  - 开关日志

这类库不是当前必须，但对工程继续长大很有帮助。

### 4. `GTest`

用途：

- 给 `geometry`、`motor_mixer`、`se3_controller` 写单元测试

建议场景：

- 你准备继续做 C++ 重构
- 你希望控制律、混控矩阵、坐标变换在修改后不退化

Ubuntu 常见安装：

```bash
sudo apt install libgtest-dev
```

### 5. `nlohmann_json` 或继续保持 `yaml-cpp`

当前配置文件已经使用 `yaml-cpp`，这对参数配置已经够用了。  
如果后面你要和别的系统交换实验结果、日志或者轨迹数据，`nlohmann_json` 也会很方便。

## 我对这个项目的建议

如果只看当前这个四旋翼 C++ 改造，最值得加的额外依赖其实只有这几类：

- `GLFW`
  - 如果你下一步想接 MuJoCo viewer
- `spdlog`
  - 如果你想把调试和运行日志做好
- `GTest`
  - 如果你准备长期维护这个 C++ 工程

其中：

- `Eigen3`、`yaml-cpp` 和 `GLFW` 我已经接进工程了
- `spdlog`、`GTest` 目前还没接进 CMake，只是推荐

## 构建

在仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
cmake --install build
```

默认约定：

- 构建产物在 `build/`
- 可执行文件在 `build/bin/quadrotor`
- 安装产物在 `build/install/`

安装时会复制仓库根目录的 `assets/` 到：

- `build/install/share/assets/`

这样安装后的 `config.yaml` 仍然可以通过相对路径找到模型文件。

## 运行

默认读取 `quadrotor/config.yaml`：

```bash
./build/bin/quadrotor
```

也可以显式传入配置文件：

```bash
./build/bin/quadrotor ./quadrotor/config.yaml
```

也可以直接切换运行模式：

```bash
./build/bin/quadrotor --viewer
./build/bin/quadrotor --headless
```

`viewer` 模式下支持：

- 鼠标左键拖动旋转视角
- 鼠标右键拖动平移视角
- 滚轮缩放
- `Space` 暂停/继续
- `Backspace` 重置仿真
- `Esc` 关闭窗口

如果本机当前没有可用图形环境，而 `config.yaml` 里又启用了 viewer，程序会自动回退到 headless 模式。

## 模型组织建议

你这个想法是对的：`quadrotor/` 更适合作为“控制器 + 仿真程序”目录，而不是“某一个具体机型的资源目录”。  
如果你后面会接多个四旋翼，我很推荐保持下面这个分层：

- `quadrotor/`
  - 放 C++ 控制、仿真、viewer、配置解析
- `assets/<airframe_name>/`
  - 放每一种机型的 `scene.xml`、`body.xml`、网格、材质

这样好处是：

- 控制代码和机型资源解耦
- 新增机型时不需要改 C++ 目录结构
- 同一个可执行文件可以切换不同机型
- 安装规则也可以直接复用整棵 `assets/`

如果你后面准备认真做多机型支持，我建议下一步再往前走一点，把 `config.yaml` 里的：

- 质量
- 力臂
- 推力系数
- 最大转速

也尽量和机型资源绑定起来，最好做到“一份机型目录 + 一份机型参数配置”。  
这样不同四旋翼之间切换时，不容易出现 XML 已经切了，但动力参数还是旧机型的情况。

## 前后差异

接入 `GLFW + MuJoCo viewer` 之前，这个 C++ 版本只有：

- 加载 XML
- 执行控制回调
- 以最快速度 headless 跑完仿真
- 在终端打印状态

接入之后，主要差异是：

- 现在可以直接看到四旋翼的实时飞行过程
- 仿真从“纯计算循环”变成了“仿真步进 + 实时渲染”的双用途主循环
- 可以通过鼠标直接观察姿态、轨迹和相机视角
- 更接近原始 Python 版 `viewer.launch(...)` 的使用体验
- 仍然保留 `--headless`，方便批量实验、日志记录和无桌面环境运行

简单说：

- `headless` 更适合跑实验、做批处理、追求速度
- `viewer` 更适合调参、看控制效果、排查姿态和轨迹问题
