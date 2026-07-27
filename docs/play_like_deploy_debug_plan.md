# Go2 Depth Deploy Play-Like 排查方案

本文记录当前阶段的排查目标和修改建议：先不考虑实机部署，只把部署仿真逐步改成接近训练侧 `play` 的数据来源和时序，用来定位 `play` 侧与部署仿真侧行为差异到底来自哪一部分。

## 背景

当前现象：

- `scene_go2_parkour_multi_from_stairs.xml` 楼梯地形下，部署仿真已经基本可以稳定完成任务。
- `scene_go2_parkour_multi_from_gaps.xml` 或 gaps 类地形下，仍然容易在路径点附近或木板间隙附近出现较大偏航、提前迈步、未正确跨过间隙等问题。
- 之前观察到 `play` 中机器人动作看起来更慢，但进一步检查发现 `play` viewer 的 `actual RT` 约为 `0.3-0.5`，说明视觉上变慢很可能来自可视化/仿真运行速度低于真实时间，而不一定代表物理参数本身更慢。

因此后续排查不应只凭肉眼速度判断，而应该记录并对比策略输入、动作输出、状态来源和深度图时序。

## 总目标

构造一个 `play-like deploy` 版本：

- 仍然运行部署仿真框架。
- 暂时不考虑是否可用于实机。
- 尽量使用 MuJoCo truth 作为部署侧状态来源。
- 逐步替换 position、yaw、waypoint、depth 等模块。
- 每一步只替换一个变量，观察 gaps 失败是否消失或明显改善。

最终要回答：

> 如果部署仿真尽量使用 MuJoCo truth 和 play-like 输入，是否还能复现 gaps 失败？

判断逻辑：

| 结果 | 说明 |
| --- | --- |
| play-like 后不再失败 | 差异主要来自部署侧状态估计、ROS2 depth、yaw/position 来源、depth 延迟或 waypoint 逻辑 |
| play-like 后仍然失败 | 差异更可能来自控制链路、PD/action apply、地形 contact、robot model 或 MuJoCo 场景物理 |

## 版本管理建议

建议在 `unitree_rl_mjlab` 仓库中新建分支：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
git checkout -b debug/play-like-deploy
```

每完成一个阶段就提交一次：

```bash
git add .
git commit -m "play-like step 1 log deploy obs timing"
```

这样每个实验阶段都可以独立回退。

## 阶段 0：不改逻辑，只增强记录

目的：先保存当前部署仿真的真实行为，作为 D0 基准。

建议记录：

| 记录项 | 用途 |
| --- | --- |
| policy step index | 对齐每个策略步 |
| wall time / sim time | 判断控制频率和实际 RT |
| depth stamp | 判断 depth 是否旧帧 |
| depth age | 判断策略使用图像相对当前状态的延迟 |
| repeated depth flag | 判断是否重复使用旧 depth |
| root pos | 对比 play 侧真实位置 |
| root quat / yaw | 对比 yaw 来源 |
| waypoint idx | 检查路径点切换时刻 |
| waypoint_delta_yaw | 检查当前目标 yaw 误差 |
| waypoint_delta_next_yaw | 检查下一个目标 yaw 误差 |
| actor obs 分段 min/max/mean | 定位是哪一段 obs 异常 |
| action 输出 | 判断策略响应 |
| joint target / joint pos / joint vel | 判断 action 到机器人是否一致 |

这一阶段不改变行为，只回答：

> 部署仿真失败前，到底哪个信号先异常？

## 阶段 1：position 改成 MuJoCo truth

目的：排除当前位置积分、SportModeState 发布、origin 对齐造成的误差。

建议增加配置开关：

```yaml
play_like:
  enabled: true
  use_mujoco_root_pos: true
```

对比：

| 当前部署侧 | play-like |
| --- | --- |
| SportModeState position 或部署侧状态 | MuJoCo root position |

如果该阶段明显改善，说明 position 来源或坐标系存在问题。

## 阶段 2：yaw 改成 MuJoCo truth

目的：排除 IMU/状态估计 yaw、yaw unwrap、坐标系转换造成的差异。

建议增加配置开关：

```yaml
play_like:
  use_mujoco_root_yaw: true
```

建议同时记录：

| 记录项 | 说明 |
| --- | --- |
| yaw_from_imu | 当前部署侧 yaw 来源 |
| yaw_from_mujoco | MuJoCo root quat 直接计算的 yaw |
| yaw_diff | 两者差值 |
| waypoint_delta_yaw_imu | 使用部署 yaw 算出的当前目标 yaw 误差 |
| waypoint_delta_yaw_mujoco | 使用 MuJoCo yaw 算出的当前目标 yaw 误差 |

如果两种 yaw 只差一点点，但在 waypoint 附近被放大成明显不同，就说明问题很可能在 yaw 来源或 yaw 坐标系。

## 阶段 3：waypoint 更新逻辑完全对齐 play

目的：排除 C++ waypoint 逻辑与训练侧 Python 逻辑不同造成的路径点附近突变。

需要逐项对齐：

| 项目 | 对齐目标 |
| --- | --- |
| waypoint 坐标 | 与训练导出或训练侧 reference terrain 一致 |
| position 输入 | 使用阶段 1 的 MuJoCo truth position |
| yaw 输入 | 使用阶段 2 的 MuJoCo truth yaw |
| reach 判断 | `distance < reach_radius` |
| reach delay | 与训练侧 `reach_goal_delay=0.1` 对齐 |
| 切点条件 | 与训练侧 `timer > delay_steps` 一致 |
| 当前 idx / next idx | 与训练侧一致 |
| yaw 计算 | `atan2(goal_y - pos_y, goal_x - pos_x) - robot_yaw` |

不要在这一阶段调大或调小 `reach_radius`。该阶段目标是验证逻辑是否一致，不是调参补偿。

## 阶段 4：depth 绕过 ROS2，改成同步 MuJoCo raycast

目的：判断 ROS2 depth 频率、延迟、缓存旧帧是否是 gaps 失败主因。

建议增加配置开关：

```yaml
play_like:
  use_sync_depth: true
```

对比：

| 当前部署侧 | play-like |
| --- | --- |
| sim 发布 depth 到 ROS2，deploy 读取最近一帧 | 每个策略步直接基于当前 MuJoCo 状态生成 depth |
| depth 可能只有 20-30Hz | depth 与策略步同步 |
| 可能重复旧帧 | 不通过 ROS2 缓存 |
| 存在通信延迟 | 逻辑上无 ROS2 延迟 |

如果该阶段明显改善，说明问题主要来自 depth ROS2 频率、延迟或旧帧。

如果仍然失败，需要继续检查 depth 图本身是否和训练侧一致。

## 阶段 5：depth 预处理完全复用训练侧规则

目的：排除 raw depth 到策略输入图像之间的处理差异。

建议保持原部署处理代码，同时提供开关：

```yaml
depth:
  preprocess_mode: astro_play
```

需要逐项确认：

| 项目 | 训练侧 | play-like deploy |
| --- | --- | --- |
| raw depth 来源 | 训练环境 camera/raycast | 同步 MuJoCo raycast |
| invalid / far 处理 | 训练侧 `camera_depth_image()` | 完全一致 |
| near/far clamp | 训练侧参数 | 完全一致 |
| resize/interpolate | 训练侧方式 | 完全一致 |
| flatten/order | 训练侧顺序 | 完全一致 |
| normalize | 训练侧规则 | 完全一致 |

该阶段需要同时保存 raw depth 和 processed depth，不能只看最终动作。

## 阶段 6：action apply / PD 控制链路检查

如果 D5 仍然失败，说明观测来源已经很接近 play，下一步应重点检查 action 到机器人之间的链路。

建议记录：

| 记录项 | 说明 |
| --- | --- |
| action | 策略原始输出 |
| default_joint_pos | 默认关节角 |
| action_scale | action 缩放 |
| target_joint_pos | 最终目标关节角 |
| joint_pos | 当前关节角 |
| joint_vel | 当前关节速度 |
| pd_kp / pd_kd | PD 参数 |
| torque | 实际力矩或控制输出 |

需要确认两侧是否都满足：

```text
target_joint_pos = default_joint_pos + action_scale * action
```

如果 action 一致，但 target 或 joint response 不一致，问题就在控制链路，而不是策略或观测。

## 实验矩阵

建议按以下顺序执行：

| 实验 | position | yaw | waypoint | depth | ROS2 | 目的 |
| --- | --- | --- | --- | --- | --- | --- |
| D0 当前 deploy | deploy | deploy | deploy | ROS2 raycast | 有 | 当前失败基准 |
| D1 | MuJoCo truth | deploy | deploy | ROS2 raycast | 有 | 检查 position 来源 |
| D2 | MuJoCo truth | MuJoCo truth | deploy | ROS2 raycast | 有 | 检查 yaw 来源 |
| D3 | MuJoCo truth | MuJoCo truth | play-like | ROS2 raycast | 有 | 检查 waypoint 逻辑 |
| D4 | MuJoCo truth | MuJoCo truth | play-like | sync raycast | 无 | 检查 ROS2 depth 延迟/频率 |
| D5 | MuJoCo truth | MuJoCo truth | play-like | sync raycast + astro preprocess | 无 | 最接近 play 的 deploy |

判断方式：

| 结果 | 可能原因 |
| --- | --- |
| D1 明显好转 | position 来源或坐标系问题 |
| D2 明显好转 | yaw 来源、yaw unwrap 或坐标系问题 |
| D3 明显好转 | waypoint 更新逻辑问题 |
| D4 明显好转 | ROS2 depth 延迟、频率或旧帧问题 |
| D5 明显好转 | depth 预处理问题 |
| D5 仍然失败 | 优先检查 action apply、PD、地形 contact、robot model |

## 当前优先怀疑项

结合已有现象，当前优先级建议如下：

1. waypoint 附近的 yaw/idx 切换逻辑。
2. gaps hfield 下 raycast/depth 频率不足或旧帧。
3. hfield gaps contact 与 box gaps 或训练侧接触模型不同。
4. action apply / PD 控制链路与 play 不一致。
5. robot model、关节阻尼、摩擦、solver、timestep 等物理设置差异。

## 注意事项

- 已经明确与训练侧对齐的设置尽量不要改。
- 每次只替换一个变量，不要同时改 yaw、depth、waypoint 和 PD。
- 所有 play-like 修改都应保留开关，不直接删除原部署逻辑。
- 记录 `actual RT`，不要仅凭肉眼速度判断物理是否不同。
- gaps 地形建议同时使用 hfield 和 box 版本做 A/B，判断是否是地形 contact 问题。

