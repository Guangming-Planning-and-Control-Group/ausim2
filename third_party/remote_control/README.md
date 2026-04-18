# Remote Control

`third_party/remote_control` 提供一个仓内 ROS2 teleop 节点：

- 订阅 `/joy`
- 发布 `/uav1/cmd_vel`
- 发布语义事件 `/uav1/teleop/event`
- 兼容旧链路：如果没有配置事件话题，起飞和重置仍可回退到 `/uav1/takeoff` 与 `/uav1/sim/reset`
- 在没有新鲜手柄输入且当前终端是 TTY 时，自动切到键盘控制

默认键盘映射：

- `w/s`：前后
- `a/d`：左右平移
- `r/f`：上下
- `j/l`：偏航
- `space`：立即清零
- `t`：起飞
- `x`：重置
- `m`：切换模式
- `q`：急停

默认手柄组合键：

- `LB + A`：起飞
- `Back + Start`：重置
- `mode_next` / `estop` 默认未绑定，需要在参数里显式配置按钮组合

直接运行：

```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node --ros-args \
  --params-file /opt/ros/humble/share/joy/config/joy-params.yaml

./build/bin/remote_control_node --ros-args \
  --params-file ./third_party/remote_control/config/xbox_like.yaml
```

repo-local launch：

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./third_party/remote_control/launch/remote_control.launch.py
```

说明：

- launch 文件会按需拉起 `joy_node`
- launch 会加载 `joy` 官方 `joy-params.yaml`，其中默认开启 `autorepeat_rate: 20.0`
- 如果单独运行 `remote_control_node`，必须另外启动 `joy_node`；否则节点只会进入键盘 fallback
- 键盘 fallback 依赖前台终端 TTY；如果通过无 TTY 的方式启动节点，键盘控制会自动禁用
