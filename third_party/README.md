# Third-Party Dependencies

本目录存放仓库直接依赖的第三方组件与配套源码。

| 组件 | 路径 | 主要内容 | 用途 |
|------|------|----------|------|
| MuJoCo Release | `mujoco-3.6.0/` | 头文件、运行时库、官方插件、官方 simulate 源码 | 提供整个仓库的 MuJoCo 编译与运行环境 |
| RayCaster Plugin Package | `mujoco_ray_caster/` | 自定义传感器插件源码、示例模型、smoke test | 构建深度相机 / 激光雷达 / 基础 ray-caster 插件 |
| Dynamic Obstacle Generator | `dynamic_obs_generator/` | 场景生成脚本、运行时辅助库、障碍物模板配置 | 为四旋翼场景生成动态障碍 XML，并提供运行期障碍物管理 |
| Remote Control | `remote_control/` | 手柄/键盘 teleop、组合键命令触发、repo-local launch 与默认参数 | 为仿真提供手柄速度控制、起飞/重置命令和键盘兜底 |

## 1. MuJoCo Release 包

`mujoco-3.6.0/` 是仓内 vendored 的 MuJoCo 3.6.0 release 包。当前工程直接消费它，不再依赖外部 MuJoCo 源码树。

关键内容：

- `mujoco-3.6.0/include/mujoco/`
  - MuJoCo 公开头文件
- `mujoco-3.6.0/lib/libmujoco.so.3.6.0`
  - MuJoCo 运行时动态库
- `mujoco-3.6.0/bin/mujoco_plugin/`
  - MuJoCo 官方插件与模型 decoder
- `mujoco-3.6.0/simulate/`
  - MuJoCo 官方查看器源码，当前仓库直接从这里编译 viewer 支持

官方插件目录当前包含：

- `libactuator.so`
- `libelasticity.so`
- `libobj_decoder.so`
- `libsdf_plugin.so`
- `libsensor.so`
- `libstl_decoder.so`

用途：

- 为 `quadrotor` 和 `scout` 提供 MuJoCo 核心库
- 提供 OBJ / STL mesh 解码能力
- 提供官方 simulate viewer 所需源码

备注：

- 当前仓库编译的 simulate 源码已禁用 screenshot PNG 导出，因此不再需要 `lodepng`

## 2. RayCaster 插件包

`mujoco_ray_caster/` 是本仓库直接编译的自定义 MuJoCo 插件源码包。

关键内容：

- `register.cc`、`ray_*`、`raycaster_src/`
  - 插件注册与核心实现
- `cmake/ResolveMuJoCo.cmake`
  - MuJoCo release 包解析逻辑
- `model/`
  - 插件示例模型
- `smoke_test.cc`
  - 插件加载与运行 smoke test
- `README.zh-CN.md`
  - 插件详细说明

构建产物：

- `build/bin/mujoco_plugin/libsensor_raycaster.so`

当前注册的插件名：

- `mujoco.sensor.ray_caster`
- `mujoco.sensor.ray_caster_camera`
- `mujoco.sensor.ray_caster_lidar`

用途：

- 为 MJCF 中的 camera / sensor 提供深度相机与激光雷达能力
- 为 `quadrotor` 传感器链路提供深度数据
- 作为 `mujoco_ray_caster_smoketest` 的核心测试对象

详细参数说明见：

- [README.zh-CN.md](mujoco_ray_caster/README.zh-CN.md)

## 3. Dynamic Obstacle Generator

`dynamic_obs_generator/` 负责四旋翼场景的动态障碍 XML 生成与辅助管理。

关键内容：

- `generate_scene_obstacles.py`
  - 根据仿真配置生成场景 XML
- `dynamic_obstacle_manager.cpp/.hpp`
  - 运行期障碍物管理辅助库
- `obstacle.yaml`
  - 动态障碍模板配置

用途：

- 在启动 `quadrotor` 前生成实际使用的场景 XML
- 让仿真主流程不直接维护复杂的障碍物拼装逻辑

## 4. 当前插件加载链路

运行时会同时加载以下两个插件目录：

- `third_party/mujoco-3.6.0/bin/mujoco_plugin`
  - MuJoCo 官方插件与 OBJ / STL decoder
- `build/bin/mujoco_plugin`
  - 本仓库构建出的 `libsensor_raycaster.so`

`em_run.sh` 会自动导出这两个目录到 `MUJOCO_PLUGIN_DIR`。
