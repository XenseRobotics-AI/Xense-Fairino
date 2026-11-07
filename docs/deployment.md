# 部署说明

本次整理只做本地构建与离线验证，未执行本页的机器人操作。部署仍沿用原实现，不会把电脑上的修改自动应用到机器人。

## 前置依赖

电脑需 Python 3.10+、SSH、项目 Python 依赖；GUI 需 GTK3/GI/cairo。电脑不需 ROS。

机器人默认 SSH 用户 `fairino`；底盘 `192.168.58.3`、ROS Domain 1；上身急停相关反馈 `192.168.58.1`、Domain 0。厂家基础系统必须已经安装并运行：

- `/opt/ros/humble`，Nav2 工作区 `/opt/nav2_ws/install`。
- `/opt/fairino_AMR_project` 指向已核对的 B 配置，含厂家硬件接口、八轴控制器和命令/反馈消息。
- 双雷达驱动、`laserscan_multi_merger`、`robot_state_publisher`、上身急停转发。
- SLAM Toolbox **2.6.10** SDK、消息和求解器；独立构建严格检查版本。
- `map_server`、AMCL，以及 Nav2 的 SmacPlanner2D、ControllerServer、生命周期管理组件，以及 CMake/C++17/make 和插件开发依赖。
- 安装步骤所需的远端 `sudo -n` 权限。

这些厂家/ROS 依赖没有复制进本工程。独立是指本地源码、配置、模型、测试和部署入口不依赖旧 toolkit。

## 入口与副作用

所有命令在本工程根目录执行，`--help` 不会连接机器人。

| 命令 | 默认行为 | `--execute` 行为 |
|---|---|---|
| `deploy control` | SSH 远端只读预检 | 备份并安装底盘/里程计配置，关闭开机自动使能 |
| `deploy mapping` | 本地说明，不连接 | 备份并安装地图、定位管理与服务 |
| `deploy slam` | 本地说明，不连接 | 下载锁定源码、机器人独立编译与隔离验证后选择版本 |
| `deploy navigation` | SSH 远端只读预检 | 独立编译并验证预瞄与直线规划插件，成功后备份安装 Nav2 配置、程序和服务 |
| `deploy priority` | 本地显示配置，不连接 | 安装已有 PAM 普通优先级修复，并验证新登录 |

地图安装器原来是直接执行。本次增加标准 `--help`/`--host`/`--user`/`--execute` 外层入口；其内部部署流程保留。

建议顺序：

```bash
bash scripts/run.sh deploy control --execute --odometry raw
bash scripts/run.sh deploy mapping --execute
bash scripts/run.sh deploy slam --execute
bash scripts/run.sh deploy navigation --execute
```

底盘配置迁移本身不重启驱动、不使能。通常在机器人静止、全部下使能后按现场流程重启机器人应用使配置生效。已经完成底盘迁移时，`deploy control --execute --odometry raw --apply-now` 可按原有严格前置检查只重启里程计；不会热更新其他硬件启动配置。

SLAM 构建先校验官方源文件 SHA-256，再应用选帧补丁；验证在本机通信的 Domain 197 进行。只安装独立运行程序，不替换系统库。下一次明确新建图才使用选定版本。

导航插件源码来自 `ros2/fairino_lookahead_controller` 和 `ros2/fairino_straight_planner`，构建到同一个独立 `navigation-plugin-builds/<时间>/install` 前缀。两个插件的构建和 pluginlib 无节点加载检查全部成功后，才让新的 runtime.sh 加载该前缀；原安装目录保留便于回退。任何一个构建失败均不覆盖现有运行文件。直线规划依赖机器人已有的 `nav2_smac_planner`，开关关闭时直接使用它的原规划结果。

导航服务运行期间拒绝覆盖更新。先正常结束目标和底盘会话、停止服务，再安装。部署服务均不自动发送导航目标；打开 GUI 也不使能。

## 参数更新

所有可调参数源位于根 [config/](../config/README.md)。部署程序读取当前配置并验证，导航参数在部署时生成；`.local/generated/nav2.yaml` 仅供本地查看。运行中的面板需重启后加载新参数。修改默认限速后更新导航部署；修改底盘上限、加速度或 jerk 时，还需更新底盘部署配置并按上述流程重启。现有服务不会热更新。

## 备份与恢复

远端每次安装的回执列出 backup 路径和文件清单，分别位于三个运行目录的 `backups/`。保留这些回执。恢复 SLAM 时需同步恢复管理程序、参数和运行版本链接，不能只替换可执行文件。

建图地图在机器人 `~/maps/manual/<name>/`；选用地图通过厂家项目 `maps/active` 指向对应目录。保存地图不自动替换默认地图，切换地图是另一项明确操作。

本地日志默认写本工程 `logs/`。源码改动后需要重新部署相应远端组件；底盘 Worker 和观察进程会在新连接时发送当前源码，现有连接不会热替换。
