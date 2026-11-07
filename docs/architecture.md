# 架构与文件索引

## 运行边界

电脑端包含 Python 命令行、GTK 地图界面、SSH 连接和输入心跳。机器人端提供 ROS 2 Humble、厂家八轴控制器、雷达驱动与合并、TF、SLAM/AMCL/Nav2。`deployment/` 将必要源码打包到机器人原有运行目录；远端不需要安装整个电脑端 Python 包。

| 环节 | 源码位置 | 输入 → 输出 |
|---|---|---|
| 建图按键与会话 | `ui/mapping_controls.py`、`ui/mapping_session.py` | 按键/焦点 → SSH 输入 |
| 底盘执行 | `chassis/transport.py`、`robot_worker.py` | 按键或导航速度 → `/fairino_keyboard/cmd_vel` |
| 轮速守护 | `odometry/odom_guard.py`、`raw_odometry.py` | 厂家原始 odom → `/odom`、`odom → base_footprint` |
| 建图管理 | `mapping/map_control_remote.py` | 明确的建图操作 → SLAM 服务与地图存档 |
| 激光 SLAM | `resources/slam_toolbox/` | `/scan`、轮速 TF → `/map`、`map → odom`、`/pose` |
| 已有地图定位 | `mapping/localization.py` | 静态地图、`/scan`、轮速 TF → AMCL 位姿和 TF |
| 全局规划与控制配置 | `navigation/profile.py`、`navigation.launch.py` | 地图、目标 → Smac 路径 → 预瞄插件 |
| 导航门控 | `navigation/velocity_gate.py`、`motion.py` | 速度和健康状态 → 有效速度或零速 |
| 可视化 | `mapping/viewer.py`、`panel.py`、`remote_stream.py` | 地图、TF、扫描与会话 → 图形界面 |
| 连接和维护 | `connection/`、`maintenance/`、`diagnostics/` | SSH、状态、日志和故障报告 |

表中的 Python 源码位置均相对 `src/fairino_slam/`。实际预瞄选点、位姿转连续速度及扫掠碰撞算法位于根 `ros2/fairino_lookahead_controller/`。

## 配置来源

根 `config/` 是统一调参入口。`configuration.py` 读取并校验配置，`navigation/profile.py` 生成 Nav2 参数；`chassis/source.py` 把底盘参数快照嵌入远端源码。安装包由构建过程自动收录同一份配置。详细字段与生效流程见 [参数配置说明](../config/README.md)。

## 坐标与数据

```text
map → odom → base_footprint → base_link → front_laser_link / back_laser_link
```

`base_footprint → base_link` 为纯 Z 高度差 0.065 m。前雷达相对 `base_link` 为 `(0.239, 0.175, 0.1455)`、yaw 45°；后雷达为 `(-0.239, -0.175, 0.1455)`、yaw −135°。这是迁入模型的现有值，本次没有实测标定。

厂家合并节点把 `/front/scan`、`/back/scan` 转到 `base_link`，输出 `/scan`。SLAM/AMCL 使用合并扫描；Nav2 障碍层使用两路原始扫描，分别保留射线原点。

原始模式保留厂家里程计的 XY、yaw、速度、协方差和时间戳，不额外积分或外推。守护程序也保留 EKF 模式，但安装器默认 raw。守护程序未找到 mode.json 时的兼容回退仍是 EKF。

## 建图、定位和导航边界

- 建图与 AMCL 互斥，防止两个模块同时提供 `map → odom`。
- 原始里程计输入持续健康一秒后启动；断流或故障后锁定停止，恢复数据不自动继续。
- 里程计 epoch 变化后，不允许继续累积旧工作地图；需要保存/结束并重新建图。
- SLAM 仍为固定 2.6.10 加选帧补丁，约 0.10 m / 5° 接纳关键帧，地图每秒更新。
- 导航为 SmacPlanner2D + 全向预瞄跟随，控制频率、欧氏预瞄距离和朝向策略由 `config/navigation.yaml` 管理，速度/加速度取 `config/chassis.yaml`。支持 `hold_initial` 保持任务开始时朝向，以及 `path_tangent` 沿路径朝向并到达指定终点朝向。算法及连续速度层见 [导航说明](navigation.md)。
- 碰撞轮廓仅覆盖 0.75 × 0.60 m 底盘，上半身动态投影仍未实现。
- 轮速控制、使能和急停保护依赖厂家系统；本工程未迁入或重写厂家底层控制器。

## 远端兼容名称

保留 `/fairino_keyboard/cmd_vel`、`/fairino_navigation/cmd_vel_raw`、`/fairino_navigation/cmd_vel`、`eight_dof_chassis_controller`、`fairino_keyboard_angle`、`.fairino_keyboard.lock`、`@KEYCTRL` 和 `MAP_VIEWER` 等协议。

机器人运行目录仍是 `~/.local/share/fairino-odometry/`、`fairino-map-control/` 和 `fairino-navigation/`。安装器继续识别原 `Managed by fairino-robot-toolkit` 标记，便于更新已有服务。这些兼容字符串不构成本地旧仓库依赖。
