# 法奥机器人控制项目

一个仓库包含底盘键控、20 轴单关节控制、身体高度与姿态联动、双臂 TCP 键控、PICO 双臂遥操、双雷达显示、故障清除和链路诊断。安装、启动、配置、模型及离线验证均在本项目内完成。

## 安装

支持 Ubuntu 24.04 x86_64、Python 3.12。首次安装需要联网。

```bash
git clone git@github.com:limfxin/fairino-robot-toolkit.git
cd fairino-robot-toolkit
sudo apt install python3-venv python3-dev python3-gi gir1.2-gtk-3.0 \
  python3-tk openssh-client curl build-essential cmake unzip libusb-1.0-0 fonts-noto-cjk
bash setup.sh all
bash run.sh test
```

`setup.sh all` 准备基础界面、离线诊断、PICO 接收服务、头显 APK、ADB 和 SDK 绑定。环境与缓存位于 `.venv/` 和 `.local/`，厂家运动学库及诊断模型随代码提供。

## 功能与项目位置

以下命令均从仓库根目录执行，路径均相对于仓库根目录。

| 功能 | 项目内位置 | 入口 |
|---|---|---|
| 底盘 WASD/JL | [apps/fairino_keyboard/](apps/fairino_keyboard/) | `bash run.sh base` |
| J1–J20 单关节控制 | [apps/fairino_upper_probe/](apps/fairino_upper_probe/) | `bash run.sh joints --group body` |
| 身体高度、前倾、侧倾、腰旋转 | [apps/fairino_upper_probe/body_control/](apps/fairino_upper_probe/body_control/) | `bash run.sh body` |
| 双臂 TCP 十二键控制 | [apps/fairino_tcp_probe/](apps/fairino_tcp_probe/) | `bash run.sh tcp --arm left` |
| PICO 安装、接收与预览 | [scripts/pico4/](scripts/pico4/) | `bash run.sh pico service` |
| 双臂遥操、插值、Home、冻结 | [scripts/pico4/fairino/](scripts/pico4/fairino/) | `bash run.sh vr` |
| PICO 设备接入与双手柄 | [src/lerobot/teleoperators/](src/lerobot/teleoperators/) | 由 PICO 入口加载 |
| 上半身和底盘清错 | [clear_errors.py](scripts/pico4/fairino/clear_errors.py) | `bash run.sh errors --execute` |
| 双雷达扫描显示 | [apps/lidar_viewer/](apps/lidar_viewer/) | `bash run.sh lidar` |
| 网络与机器人状态 | [tools/](tools/) | `bash run.sh network` / `status` |
| 雷达与手腕离线诊断 | [diagnostics/](diagnostics/) | `bash run.sh diagnostics --help` |
| 模型、网格和样例 | [models/](models/)、[data/](data/) | 各模块自动按项目路径读取 |
| 全部离线验证 | [tests/](tests/) | `bash run.sh test` |

`base / joints / body / tcp` 普通启动会使能；`vr` 会使能并先回双臂 Home。只读检查用对应 `--check`，离线界面用 `--demo`。全部命令见 `bash run.sh help`。

## 开机后的故障检查

开机或重启后可能仍有驱动器故障。已记录过全局 `0x0E10FF01`、关节 `J15=30`；`30` 为各轴十进制错误码，现有故障表将其归为 EtherCAT 通信类故障。具体轴与原因以当次反馈为准。

```bash
bash run.sh errors
# 有故障时，先退出控制程序，确认静止、全部未使能、急停释放
bash run.sh errors --execute
bash run.sh errors
```

清错只请求一次复位，并复查结果。确认全局和各轴错误码归零后再启动控制；故障复发时保留日志排查。

## PICO 双臂遥操

```bash
bash run.sh pico service  # 第一个终端保持运行
bash run.sh pico monitor # 另一个终端先确认数据，随后退出 monitor
bash run.sh vr
```

头显填写电脑可达的局域网地址，开启 Controller Tracking 和 Send。右手柄控制左臂，左手柄控制右臂；Grip 握住跟随、松开保持。A/X 冻结旋转，B/Y 冻结位移；左摇杆上下调灵敏度、按压暂停；右摇杆按压回 Home。

`vr` 指定 450 Hz TCP 插值、IK 和下发，500 Hz 接收轮询，100 Hz 状态回传，12 ms 输入缓冲。链路为 TCP 位移线性插值／姿态 SLERP → 厂家 IK → 有线 SSH → 机器人 ROS 控制器 → EtherCAT。实际频率以日志为准。

## 说明与配置

- [安装与运行](docs/安装与运行.md)
- [功能汇总与模块索引](docs/功能汇总.md)
- [网络配置](docs/机器人网络配置与连接说明.md)
- [遥操与延迟](docs/PICO遥操有线方案与延迟说明.md)
- [雷达与模型](docs/雷达与模型.md)
- [故障排查](docs/故障排查.md)

双臂 Home 在 [home_pose.json](scripts/pico4/fairino/home_pose.json)，身体参数在 [body_config.json](apps/fairino_upper_probe/body_control/body_config.json)，依赖清单在 [requirements/](requirements/)。第三方许可见 [资源说明](THIRD_PARTY_NOTICES.md)。
