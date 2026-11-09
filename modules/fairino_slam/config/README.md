# 参数配置

日常调参只编辑本目录。源码运行和 editable 安装直接读取这里；构建 wheel 时自动收录同一份内容，源码中不保留另一份配置。

| 文件 | 参数 |
|---|---|
| [chassis.yaml](chassis.yaml) | 建图/导航默认速度、底盘速度上限、加速度、jerk、轮组限制、反馈/心跳超时 |
| [navigation.yaml](navigation.yaml) | Nav2 控制频率、采样、到点容差、代价地图、避障、轮廓留量 |
| [mapping.yaml](mapping.yaml) | SLAM 地图分辨率、更新周期、关键帧距离/角度、激光范围 |
| [localization.yaml](localization.yaml) | AMCL 粒子数、运动更新阈值、激光模型 |
| `behavior_trees/` | 导航行为树；修改时保持现有插件支持和 XML 有效 |

## 速度与加速度

`chassis.yaml` 中的参数含义：

| 字段 | 当前值 | 用途 |
|---|---|---|
| `speed.default_linear` | 0.10 m/s | 导航 XY 合速度上限、建图键控初始平移速度 |
| `speed.default_angular` | 0.20 rad/s | 导航旋转速度上限、建图键控初始旋转速度 |
| `speed.max_linear` | 0.20 m/s | 底盘软件平移上限、建图调速上限 |
| `speed.max_angular` | 0.40 rad/s | 底盘软件旋转上限、建图调速上限 |
| `limits.linear.max_acceleration` | 0.20 m/s² | 底盘平移加速度及导航 XY 加减速度幅值 |
| `limits.angular.max_acceleration` | 0.40 rad/s² | 底盘旋转加速度及导航角加减速度幅值 |

速度与轮组几何包络仍受现有校验约束。机械转向限位、关节布局、通信协议等实现约束仍由代码维护。

导航的速度和加速度由 `chassis.yaml` 自动注入。`navigation.yaml` 的 `controller_frequency` 决定 `control_period = 1 / controller_frequency`；控制周期须短于 0.6 秒速度指令超时，门限统一定义在 `src/fairino_slam/navigation/runtime_policy.py` 的 `COMMAND_TIMEOUT`。底盘轮廓来自 URDF，结合 `geometry` 计算碰撞留量及膨胀半径。不要再在导航模板中手工加入这些派生字段，校验会指出冲突。

当 `FollowPath.heading_mode=hold_initial` 时，全局轮廓使用加边距后底盘的外接圆，由 64 边形近似；生成时补偿弦长及浮点解析误差，确保有效内切半径不小于外接圆半径。`path_tangent` 使用原来的全局方形包络。局部控制仍使用真实矩形轮廓。半径随 URDF 尺寸及 `geometry.footprint_padding` 自动计算，不单独填写 `robot_radius`。当前有效半径约 0.508 m，`inflation_radius=0.75 m` 是障碍物代价向外衰减的范围，不是车身半径。

## 直线优先开关

`navigation.yaml` → `nav2.planner_server.ros__parameters.GridBased.straight_line_enabled`：

- `true`（默认）：无障碍直达，否则先绕障搜索，再安全合并多余拐点。
- `false`：直接使用原 SmacPlanner2D，不做上述优化，其他原规划参数继续生效。

开关对两种朝向策略都有效。保留现有碰撞轮廓及膨胀半径；预瞄距离和速度参数不受此开关影响。重新部署导航后生效，具体检查范围与限制见 [导航说明](../docs/navigation.md#直线优先全局规划)。

## 朝向策略

编辑 `navigation.yaml` 的 `nav2.controller_server.ros__parameters.FollowPath.heading_mode`：

| 值 | 行驶朝向 | 目标朝向输入 |
|---|---|---|
| `hold_initial`（当前默认） | 保持发送任务时的车身朝向 | GUI 禁用；CLI 的旧 `--yaw` 会被忽略 |
| `path_tangent` | 跟随前方路径方向，末段转向输入的终点朝向 | GUI 必填，单位度；CLI `--yaw` 必填，单位弧度 |

目标朝向为 map 绝对朝向，0° 沿 +X，90° 沿 +Y。末段过渡复用预瞄距离和位置容差，角速度/角加速度仍取 `chassis.yaml`。切换策略后需重新部署导航并重启面板；电脑端与机器人端策略不匹配会拒绝发送目标。

建图面板的滑块只改变当前键控会话速度，不保存 YAML，也不改变导航限速。

## 修改与生效

1. 结束当前控制会话并关闭电脑端面板，再编辑 YAML；运行中的程序不会热更新参数。
2. 在仓库根目录执行离线检查；需要查看完整 Nav2 参数时生成文件：

   ```bash
   bash scripts/run.sh check
   bash scripts/run.sh build-config
   bash scripts/check.sh
   ```

   生成文件为 `.local/generated/nav2.yaml`。它可以删除或重新生成，不是调参入口。`build-config --check` 只校验源配置，不写文件。部署导航时直接从当前配置生成参数，不依赖这份本地文件。

3. 重新启动电脑端。底盘新连接使用该进程加载的参数快照；远端安装程序使用部署时的配置。
4. 按 [部署说明](../docs/deployment.md) 同步受影响的机器人组件：导航参数/默认限速需要更新导航部署；底盘上限、加速度、jerk 等还需更新底盘配置并按现场流程重启使其生效；SLAM/AMCL 修改需同步对应建图、定位运行配置并重新启动相关会话。

只改本地 YAML 或执行 `build-config` 不会连接机器人，也不会更新已经运行的机器人服务。

## TF 门控与请求重试

导航组合 TF 的年龄门限和命令使用的有限重试规则统一维护在
[`runtime_policy.py`](../src/fairino_slam/navigation/runtime_policy.py)，当前 TF 门限为 1.0 秒（严格小于）。
预览及发送目标前的朝向检查使用同一门限；未来时间戳仍只允许原有的 0.05 秒偏差。
这些规则与模块一同部署到机器人。`navigation.yaml` 的 `transform_tolerance` 是 Nav2 插件的另一项参数。

行驶中全局重规划失败后每 1 秒持续重试，行为树位于本目录的 `behavior_trees/`；
重试范围、生效方式和停止条件见 [导航说明](../docs/navigation.md)。

## 校验边界

底盘与导航配置会检查缺失/未知字段、重复字段、数值类型、有限性、范围和关键参数之间的约束；SLAM/AMCL 保留 ROS 原生结构，并检查基本范围。拼写错误或缺失配置会明确报错，不静默退回旧参数。

导航模板对应直线优先规划器（复用 SmacPlanner2D）+ 全向预瞄控制器实现。当前 0.20 m 预瞄、6 Hz 及两种朝向策略的具体设置见 [导航说明](../docs/navigation.md)。新增算法、ROS 插件或新的模板字段时，应同时更新配置校验、部署依赖和测试。离线校验不能替代实际 ROS 插件加载及机器人运动验证。
