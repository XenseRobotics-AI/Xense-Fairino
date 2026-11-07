# 迁移记录

迁移日期：2026-11-04（Asia/Shanghai）。目标为独立目录 `/Users/zbk/XenseRobo_work/fairino_slam`。迁移来源是原工作区当前文件，包含已修改与未提交源码，不仅是 Git HEAD。

## 保留与归类

| 原位置 | 新位置（相对 src/fairino_slam） |
|---|---|
| `apps/map_viewer/` | `mapping/`；配置到 `resources/config/`，SLAM 补丁到 `resources/slam_toolbox/` |
| `apps/navigation/` | `navigation/`；参数及行为树到 `resources/config/` |
| `apps/fairino_keyboard/control_core.py`、`robot_worker.py` | `chassis/` |
| 原键控 Connection/SSH 打包 | `chassis/transport.py` |
| 原键控 Window | `ui/mapping_controls.py`，嵌入建图界面 |
| `apps/workbench/odom_guard.py`、`raw_odometry.py` | `odometry/` |
| `tools/map_mode.py`、`navigation_mode.py` | `mapping/client.py`、`navigation/client.py` |
| 安装、构建与远端验证脚本 | `deployment/` |
| 里程计、控制链路与机器人状态诊断 | `diagnostics/` |
| `tools/ssh_read.py`、workbench 路由工具 | `connection/` |
| 原 `clear_errors.py`、`fault_reset_remote.py` | `maintenance/faults.py`、`fault_reset_remote.py` |
| `models/amr_8dof.urdf` | `resources/models/`，生成独立校验清单 |

测试按 chassis、mapping、navigation、odometry、maintenance、ui 分类。新增的分发测试验证包导入、部署文件闭包，以及机器人端没有本地 Python 包时的平铺运行方式。

## 有意排除

VR/PICO、双臂遥操、关节/TCP/身体姿态控制、夹爪、Lerobot、整机网格查看、无关模型和图片、旧虚拟环境、凭据、地图数据和大量原始日志均未迁入。

用户补充建图需要键控后，保留建图专用按键操作，和导航复用同一底盘执行模块。它不作为独立旧键控应用迁入。

## 为独立运行所做的适配

- 统一为 `src/fairino_slam` 包和惰性 CLI 分发；移除旧 `apps`、`scripts.pico4`、`tools` 导入及父工程搜索路径。
- 配置、行为树和模型集中为包资源；本地日志由 paths.py 管理。
- SSH 打包、机器人平铺模块、独立构建验证支持各自导入方式；保持机器人已有协议和服务名称。
- 地图安装器增加明确 `--execute`，帮助入口完全离线。
- 原 Window 复用到地图界面，并接入同一底盘占用管理及退出等待。
- 不改 SLAM、AMCL、DWB 算法，不改速度/反馈保护阈值，不实现尚在讨论的预瞄控制。

原仓库已有文件通过迁移前后 SHA-256 比对，不应发生变化。迁移过程与最终离线结果见 validation.md 和 validation.json。
