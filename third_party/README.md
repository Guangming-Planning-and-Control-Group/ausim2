# Third-Party Dependencies

本目录包含项目使用的第三方依赖项。

| 组件 | 路径 | 用途 |
|------|------|------|
| MuJoCo Simulate | `mujoco-3.6.0/simulate/` | MuJoCo 官方查看器源码（直接从 release 包编译） |
| RayCaster Plugins | `mujoco_ray_caster/` | 激光雷达/深度相机传感器插件 |
| Dynamic Obstacle Generator | `dynamic_obs_generator/` | 动态障碍物生成系统 |

---

## 1. MuJoCo Simulate

当前工程直接编译 `mujoco-3.6.0/simulate/` 下的官方查看器源码，不再保留额外抓取的源码副本。

```
mujoco-3.6.0/simulate/
├── simulate.cc    # 主程序
├── simulate.h     # 头文件
└── ...
```

### 用途
功能完整的 MuJoCo 交互界面，支持模型加载、传感器绘图、性能分析。

### 备注
当前构建中已禁用 screenshot PNG 导出，因此不再需要 `lodepng` 依赖。

---

## 2. mujoco_ray_caster

绑定在 camera 上的 raycaster 传感器插件，基于 mj_ray 实现，参数尽量贴近 isaaclab。支持直接使用 C++ API。

**文档语言**: [English](mujoco_ray_caster/README.md) | [简体中文](mujoco_ray_caster/README.zh-CN.md)

### sensors

| 传感器 | 说明 |
|--------|------|
| `mujoco.sensor.ray_caster` | 基础射线caster |
| `mujoco.sensor.ray_caster_camera` | 深度相机 |
| `mujoco.sensor.ray_caster_lidar` | 激光雷达 |

### Build

需要 clone 与目标版本一致的 MuJoCo：

```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git
cd ..
```

修改 `mujoco/CMakeLists.txt`，添加：
```cmake
add_subdirectory(plugin/mujoco_ray_caster)
```

编译：
```bash
mkdir build && cd build
cmake ..
cmake --build . -j<线程数>
cd bin && mkdir mujoco_plugin && cp ../lib/*.so ./mujoco_plugin/
```

测试：
```bash
./simulate ../../plugin/mujoco_ray_caster/model/ray_caster.xml
./simulate ../../plugin/mujoco_ray_caster/model/ray_caster2.xml
```

---

### MJCF 配置详解

#### 2.1 基础配置 (base config)

##### SensorData

**`sensor_data_types: string list(n)`**

通过下划线组合数据模式，会按顺序拼接数据到 `mjData.sensordata`。

| data_type | 说明 |
|-----------|------|
| `data` | 距离（米） |
| `image` | [0,255] 图像数据（开启噪声后可选原图/噪声图） |
| `normal` | [0,1] 归一化法线数据 |
| `pos_w` | 世界坐标系下射线命中点（未命中或超出为 NAN） |
| `pos_b` | 传感器坐标系下射线命中点 |
| `inv` | 反转数据 |
| `inf_zero` | 射线未检测到时给定 0（默认给 inf_max） |
| `noise` | 数据是否带噪声 |
| `distance_to_image_plane` | 到图像平面距离 |
| `image_plane_image` | 图像平面图像 |
| `image_plane_normal` | 图像平面法线 |

**修饰符组合规则**:

| cfg \ data_type | data | image | normal | distance_to_image_plane | image_plane_image | image_plane_normal | pos_w | pos_b |
|-----------------|------|-------|--------|-------------------------|-------------------|---------------------|-------|-------|
| `inv`           | ✘    | ✔     | ✔      | ✘                       | ✔                 | ✔                   | ✘     | ✘     |
| `inf_zero`      | ✔    | ✔     | ✔      | ✔                       | ✔                 | ✔                   | ✘     | ✘     |
| `noise`         | ✔    | ✔     | ✔      | ✔                       | ✔                 | ✔                   | ✘     | ✘     |

**示例**:
```xml
<config key="sensor_data_types" value="data data_noise data_inf_zero inv_image_inf_zero noise_image pos_w pos_b normal inv_normal" />
```

**`dis_range: real(6), "1 1 1 0 0 0"`**
- 测距范围

**`geomgroup: real(6), "1 1 1 0 0 0"`**
- 检测哪些几何体组

**`detect_parentbody: real(1), "0"`**
- 是否检测传感器父 body

##### VisVisualize 可视化配置

**`draw_deep_ray: real(7), "1 5 0 1 0 0.5 1"`**
- 绘制射线: `ratio width r g b a edge`

**`draw_deep_ray_ids: real(6+n), "1 5 1 1 0 0.5 list"`**
- 绘制指定 id 的射线: `ratio width r g b a id_list`

**`draw_deep: real(6), "1 5 0 0 1 0.5"`**
- 绘制测量深度的射线: `ratio width r g b a`

**`draw_hip_point: real(6), "1 0.02 1 0 0 0.5"`**
- 绘制射线命中点: `ratio point_size r g b a`

**`draw_normal: real(6), "1 0.02 1 1 0 0.5"`**
- 绘制命中点法线: `ratio width r g b a`

**示例**:
```xml
<config key="draw_deep_ray" value="1 5 0 1 1 0.5 1" />
<config key="draw_deep_ray_ids" value="1 10 1 0 0 0.5 1 2 3 4 5 30" />
<config key="draw_deep" value="1 5 0 1 0" />
<config key="draw_hip_point" value="1 0.02" />
<config key="draw_normal" value="1 5 " />
```

##### Noise 噪声配置

**`noise_type: [uniform, gaussian, noise1, noise2, stereo_noise]`**

**`noise_cfg: n`** — 噪声参数：

| noise_type | noise_cfg 参数 |
|------------|----------------|
| `uniform` | `low high seed` |
| `gaussian` | `mean std seed` |
| `noise1` | `low high zero_probability seed` |
| `noise2` | `low high zero_probability min_angle max_angle low_probability high_probability seed` |
| `stereo_noise` | `pow seed` |

**noise1**: 在均值噪声基础上增加随机置 0

**noise2**: 根据射线入射角度判断的噪声。在入射角 [90°, 180°] 范围内，置 0 概率从 `low_probability` 线性插值到 `high_probability`。可模拟接近掠射角时的数据丢失现象。

**stereo_noise**: 双目深度相机噪声，需要 `mujoco version >= 3.5.0`。噪声模型见 [compute.zh-CN.md](mujoco_ray_caster/compute.zh-CN.md#3-stereo-noise--min-energy)。

##### Other 其他配置

**`compute_time_log: real(1), "0"`**
- 打印计算时间

**`n_step_update: real(1), "1"`**
- 隔 n_step 计算一次

**`num_thread: real(1), "0"`**
- 增加线程数计算 ray，提升性能。使用该参数时线程较多需每次重启程序

**`lossangle: real(1), "0"`**
- 射线丢失阈值（度），命中点到相机向量与法线夹角大于此值时射线丢失。范围 (0, 90)。需要 `mujoco version >= 3.5.0`

#### 2.2 传感器类型配置

##### RayCaster

**`resolution: real(1), "0"`** — 分辨率

**`size: real(2), "0 0"`** — 尺寸（米）

**`type: [base, yaw, world]`**
- `base`: 自坐标系相机 lookat
- `yaw`: 自坐标系 yaw，世界 z 向下
- `world`: 世界坐标系 z 向下

##### RayCasterCamera

**`focal_length: real(1), "0"`** — 焦距（cm）

**`horizontal_aperture: real(1), "0"`** — 水平尺寸（cm）

**`vertical_aperture: real(1), "0"`** — 垂直尺寸（cm）

**`size: real(2), "0 0"`** — `h_ray_num, v_ray_num`

**`baseline: real(1), "0"`** — 双目相机基线距离（米），可还原双目深度相机的重影和边缘阴影现象

**`min_energy: real(1), "0"`** — 射线丢失阈值，需要 `mujoco version >= 3.5.0`

##### RayCasterLidar

**`fov_h: real(1), "0"`** — 水平视场角（度）

**`fov_v: real(1), "0"`** — 垂直视场角（度）

**`size: real(2), "0 0"`** — `h_ray_num, v_ray_num`

#### 2.3 获取数据

`mjData.sensordata` 包含所有数据，`mjData.plugin_state` 储存 info：
- `h_ray_num, v_ray_num, list[data_point, data_size]`
- `data_point` 是相对于该传感器总数据的位置偏移

**C++ 示例**:
```cpp
std::tuple<int, int, std::vector<std::pair<int, int>>>
get_ray_caster_info(const mjModel *model, mjData *d,
                    const std::string &sensor_name) {
  std::vector<std::pair<int, int>> data_ps;
  int sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1) {
    std::cout << "no found sensor" << std::endl;
    return std::make_tuple(0, 0, data_ps);
  }
  int sensor_plugin_id = m->sensor_plugin[sensor_id];
  int state_idx = m->plugin_stateadr[sensor_plugin_id];

  for (int i = state_idx + 2;
       i < state_idx + m->plugin_statenum[sensor_plugin_id]; i += 2) {
    data_ps.emplace_back(d->plugin_state[i], d->plugin_state[i + 1]);
  }
  int h_ray_num = d->plugin_state[state_idx + 0];
  int v_ray_num = d->plugin_state[state_idx + 1];
  return std::make_tuple(h_ray_num, v_ray_num, data_ps);
}
```

**Python 示例**:
```python
def get_ray_caster_info(model: mujoco.MjModel, data: mujoco.MjData, sensor_name: str):
    data_ps = []
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id == -1:
        print("Sensor not found")
        return 0, 0, data_ps
    sensor_plugin_id = model.sensor_plugin[sensor_id]
    state_idx = model.plugin_stateadr[sensor_plugin_id]
    state_num = model.plugin_statenum[sensor_plugin_id]
    for i in range(state_idx + 2, state_idx + state_num, 2):
        if i + 1 < len(data.plugin_state):
            data_ps.append((int(data.plugin_state[i]), int(data.plugin_state[i + 1])))
    h_ray_num = int(data.plugin_state[state_idx]) if state_idx < len(data.plugin_state) else 0
    v_ray_num = int(data.plugin_state[state_idx + 1]) if state_idx + 1 < len(data.plugin_state) else 0
    return h_ray_num, v_ray_num, data_ps
```

#### 2.4 Demo

**C++**:
```bash
cd demo/C++ && mkdir build && cd build && cmake .. && make
./sensor_data
```

**Python**:
```bash
pip install mujoco-python-viewer
cd demo/Python
python3 sensor_data_viewer.py
python3 view_launch.py
python3 stereo_camera.py
```

**ROS2** (需要安装 cyclonedds-cpp):
```bash
sudo apt update && sudo apt install ros-<distro>-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd demo/ROS2/C++ && mkdir build && cd build && cmake .. && make
./sensor_data
```

---

## 3. dynamic_obs_generator

动态障碍物生成系统，在 MuJoCo 仿真环境中动态移动障碍物。

### 架构

```
dynamic_obs_generator/
├── CMakeLists.txt              # 构建配置
├── obstacle_config.hpp/cpp     # 障碍物配置结构和解析
├── dynamic_obstacle_manager.hpp/cpp  # 运行时障碍物管理器
├── obstacle.yaml               # 障碍物运动参数配置
├── generate_scene_obstacles.py # 场景障碍物生成工具
└── README.md                   # 本文档
```

### 工作原理

1. **生成**: `generate_scene_obstacles.py` 在启动仿真前基于 `obstacle.yaml` 向场景 XML 注入 `dynamic_obs_*` geoms
2. **扫描**: `DynamicObstacleManager` 在初始化时扫描模型中所有 `dynamic_obs_*` 命名的 geom
3. **记录**: 读取每个 geom 的初始位置、尺寸和运动参数
4. **轨迹播放**: 默认对非物理障碍按深度传感器节拍直接播放预定位置；如果障碍参与碰撞，则退回物理步频更新
5. **碰撞**: 障碍物碰到边界时仅反转对应轴向速度

### 使用方法

#### 1. 在场景 XML 中添加障碍物

在 MuJoCo XML 的 `<worldbody>` 内添加带有 `dynamic_obs_` 前缀名称的 geom：

```xml
<worldbody>
  <!-- 动态障碍物示例 -->
  <geom name="dynamic_obs_0" type="cylinder" size="0.3 0.5"
        pos="1.0 2.0 1.0" rgba="0.7 0.2 0.2 1" contype="0" conaffinity="0"/>
  <geom name="dynamic_obs_1" type="box" size="0.3 0.3 0.5"
        pos="2.0 1.0 0.5" rgba="0.2 0.7 0.2 1" contype="0" conaffinity="0"/>
  <geom name="dynamic_obs_2" type="sphere" size="0.4"
        pos="0.5 0.5 1.0" rgba="0.2 0.2 0.7 1" contype="0" conaffinity="0"/>
</worldbody>
```

**命名规则**: geom 名称必须以 `dynamic_obs_` 开头，后跟数字

**属性说明**:
- `type`: 形状类型 - `cylinder`, `box`, `sphere`, `cube`
- `size`: 尺寸（MuJoCo half-size 格式）
- `pos`: 初始位置 (x, y, z)
- `rgba`: 颜色
- `contype="0" conaffinity="0"`: 默认无物理碰撞（仅可视化）

#### 2. 启用动态障碍物

在 `sim_config.yaml` 中：

```yaml
dynamic_obstacle:
  enabled: true
  config_path: "../../third_party/dynamic_obs_generator/obstacle.yaml"
```

#### 3. 配置运动参数

编辑 `obstacle.yaml`：

```yaml
# 随机种子
random_seed: 42

# 是否启用运行时运动
dynamic: true

# 是否输出逐障碍初始化日志和完整调试信息
debug: false

# 运动模式: "2d" 或 "3d"
mode: "2d"

# 统一 box / cube 边长
box_size: 0.1

# 生成范围
range:
  x_min: -3.0
  x_max: 3.0
  y_min: -3.0
  y_max: 3.0
  z_min: 0.0
  z_max: 2.0

# 障碍物数量（会自动扫描场景中的 dynamic_obs_* geoms）
obstacle_count: 10

# 运动速度
min_speed: 0.0    # 最小速度 (m/s)
max_speed: 0.5   # 最大速度 (m/s)
```

### 配置参数详解

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `random_seed` | int | 42 | 随机种子，用于重现 |
| `dynamic` | bool | true | 是否启用运行时障碍运动 |
| `debug` | bool | false | 是否打印逐障碍初始化日志和完整调试信息 |
| `mode` | string | "2d" | "2d" 或 "3d" 运动模式 |
| `box_size` | float | 0.5 | box / cube 共用边长 |
| `obstacle_count` | int | 10 | 最多扫描的障碍物数量 |
| `min_speed` | float | 0.0 | 最小速度 |
| `max_speed` | float | 0.5 | 最大速度 |
| `range.x_min/max` | float | -3.0/3.0 | X 范围边界 |
| `range.y_min/max` | float | -3.0/3.0 | Y 范围边界 |
| `range.z_min/max` | float | 0.0/2.0 | Z 范围边界 |

### 运动模式

- **2D 模式**: 障碍物在 XY 平面移动，Z 坐标保持固定（用于地面环境）
- **3D 模式**: 障碍物在 XYZ 三维空间移动（用于空中环境）

### Python 场景生成工具

```bash
python3 third_party/dynamic_obs_generator/generate_scene_obstacles.py \
    --sim-config quadrotor/cfg/sim_config.yaml \
    --print-output-path
```

支持的形状: `cylinder`, `box`, `sphere`

### 限制

- 障碍物 geom 必须在 XML 场景中预定义（MuJoCo 不支持运行时创建新 geom）
- 障碍物名称必须以 `dynamic_obs_` 开头
- 如果场景中没有匹配的 geom，管理器会输出警告
