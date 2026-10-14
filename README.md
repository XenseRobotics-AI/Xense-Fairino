# 法奥轮式人形机器人调试工具

底盘、身体、双臂 TCP、PICO 双臂遥操、雷达显示、故障清除及链路诊断的当前源码汇总。面向 Ubuntu 24.04 x86_64，电脑通过 SSH 使用机器人现有 ROS 2 Humble 服务。

## 首次准备

```bash
sudo apt install python3-venv python3-dev python3-gi gir1.2-gtk-3.0 \
  python3-tk build-essential cmake unzip curl libusb-1.0-0 openssh-client
bash setup.sh base
bash setup.sh pico  # 使用 PICO 时需要；按固定清单下载官方 PC Service/APK
```

启动命令可直接在仓库根目录执行。PICO 接收服务需先运行；如果电脑已有对应后台服务，不必重复启动。

```bash
bash run.sh help
bash run.sh network
bash run.sh errors                         # 只读故障检查
bash run.sh errors --execute               # 单次清错：上半身→底盘，不使能
bash run.sh base                           # 底盘 WASD / JL，启动回正
bash run.sh joints --group left            # 左臂 J1–J7 单轴
bash run.sh joints --group right           # 右臂 J8–J14 单轴
bash run.sh joints --group body            # 头部/身体 J15–J20 单轴
bash run.sh body                           # 身体高度和姿态联动
bash run.sh tcp --arm left                 # 双臂 TCP 键控
bash run.sh lidar                          # 双雷达扫描图
bash run.sh pico service                   # PICO 接收服务，保持运行
bash run.sh pico monitor                   # 原始手柄数据
bash run.sh vr                             # 面对面双臂遥操，会使能并先回 Home
bash run.sh test                           # 离线验证
```

`base / joints / body / tcp` 普通启动会使能，相关 `--check` 或 `--demo` 模式见各程序帮助。

## 开机后的故障检查与清除

机器人开机或重启后，可能存在尚未清除的驱动器故障，使键控或遥操启动中止。此前实际记录过 **全局故障 `0x0E10FF01`、关节故障 `J15=30`**；其中 `30` 是各轴 `errorcode` 的十进制值，按现有故障表属于驱动器 EtherCAT 通信类故障。具体故障轴和错误码以本次反馈为准；出现于开机后并不能确定故障原因就是启动时序。

等待机器人服务和反馈稳定后，在仓库根目录执行：

```bash
bash run.sh errors              # 先只读检查上半身和底盘
# 有故障时：退出遥操/键控，确认机器人静止、全部未使能、急停已释放
bash run.sh errors --execute    # 单次清错，先上半身后底盘，不使能、不运动
bash run.sh errors              # 再次核对全局与各轴错误码均为 0
```

确认清除成功后，再启动需要的键控或遥操。无故障时无需复位；脚本发现控制器仍被占用、关节仍使能或急停未释放会拒绝清错。若故障无法清除或很快重现，保留日志排查，不循环清错。

## 当前遥操

PICO → Wi-Fi → 电脑映射/灵敏度/冻结 → TCP 位移线性插值与姿态 SLERP → 厂家 IK → 有线 SSH → 机器人 ROS 控制器 → EtherCAT 关节驱动。

`vr` 快捷入口指定 450 Hz 插值/IK/下发、500 Hz 接收轮询、100 Hz 状态回传、12 ms 输入缓冲；实际速率看日志。原 `pico fairino` 入口默认 90/200/100 Hz。机器人执行单点位置保持，已移除旧的 30 ms 关节追赶段。

右手柄控制左臂，左手柄控制右臂；握 Grip 跟随、松开保持。A/X 冻结旋转，B/Y 冻结位移；左摇杆上下调灵敏度、按压暂停；右摇杆按压回 Home。启动默认先回双臂 Home，可加 `--no-auto-home` 跳过。头显必须打开 Controller Tracking 和 Send。

## 目录

| 路径 | 内容 |
|---|---|
| `run.sh / setup.sh` | 根目录启动/安装入口 |
| `fairino_toolkit/apps/` | 底盘、单轴、身体、TCP、雷达 |
| `fairino_toolkit/scripts/pico4/` | PICO 接收、遥操、清错、延迟测量 |
| `fairino_toolkit/src/lerobot/` | 本地 LeRobot 源码，含 PICO/双手柄及 C++ 绑定 |
| `fairino_toolkit/models/` | 当前采用的模型 |
| `fairino_toolkit/vendor/` | 运行必需的厂商库、版本和许可信息 |
| `diagnostics/` | 雷达、电池等专项诊断源码 |
| `文档/` | 功能、网络及历史延迟汇总 |

上半身 `192.168.58.1`（Domain 0），底盘 `192.168.58.3`（Domain 1）；SSH 用户 `fairino`。电脑有线配置无冲突的 `.58.x/24`，PICO 填电脑可达的局域网地址。具体网卡、地址和路由须按所在电脑核对。仓库中的登录说明只保留输入提示。

完整说明：[功能汇总](文档/功能汇总.md)、[网络说明](文档/机器人网络配置与连接说明.md)、[有线方案与延迟说明](文档/PICO遥操有线方案与延迟说明.md)。电池、力控、灵巧手控制仍有未完成部分，见功能汇总。

## 依赖与验证

安装包、虚拟环境、下载缓存、原始大日志和厂家参考资料另行保存。控制功能需要的两个 SDK 动态库保留在仓库。第三方许可见 [资源说明](fairino_toolkit/THIRD_PARTY_NOTICES.md) 和 `fairino_toolkit/vendor/lerobot_source/LICENSE`；通用 LeRobot 模块的存在不代表所有其他机器人功能均已安装或验证。

当前控制代码已有 225 项离线测试。具体测试结果在 `fairino_toolkit/tests/`；离线结果不等于完整真机动作验证或空白系统安装验证。
