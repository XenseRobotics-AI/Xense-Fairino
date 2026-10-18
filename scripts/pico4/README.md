# PICO 接收与双臂遥操

模块位于 `scripts/pico4/`。全部命令从仓库根目录运行。

夹爪遥操在工作台「PICO 遥操」页显式勾选「同时使能 TacCap 夹爪遥操」后生效。
命令行使用 `bash run.sh vr --grippers`，原 `bash run.sh vr` 仍只启用上半身。
Grip 保持双臂跟随；食指 Trigger 按压量线性映射为夹爪闭合程度，
`开度 = 1 - Trigger`。与手臂保持同一手柄对应关系：默认镜像配置中右手柄控制左臂/左爪。
左右 SN 留空按 SDK Left/Right 识别，也可分别指定固件 SN；缺少一侧不复用另一侧设备。

启动、Home、暂停和失去有效输入期间夹爪保持；启动与恢复时先松开 Trigger，再按压开始。
按压后松开对应全开。暂停取消当前夹爪行程但保持使能，退出请求夹爪下使能。
夹爪网络与串口控制在独立线程/进程运行，不阻塞双臂高频控制。
调试页和遥操共用 [夹爪接口](../../apps/gripper_control/README.md)。

工作台还可显式勾选「启用身体竖直升降」，或使用
`bash run.sh vr --body-lift`（两者一起用：`bash run.sh vr --grippers --body-lift`）。
右摇杆上下控制竖直速度，回中保持当前高度；启动、暂停、Home 后需回中才能再次升降。
不需要握 Grip。右摇杆按压仍回双臂 Home，左摇杆上下仍调整双臂灵敏度。

升降只改变 J18–J20，复用身体程序的 `body_core.py` 和 `body_config.json`：
标定低位至高位约 425 mm，保持起始水平偏移与胸部倾角、头部、腰转和侧倾。
默认竖直速度 15 mm/s，同时按身体配置默认 5°/s 限制折叠关节指令变化率。
侧倾时下降到不允许侧倾的高度会停止，需先用身体面板回正侧倾。
双臂与升降合成同一二十轴目标，由同一个机器人端控制实例下发；
摇杆回中/输入断流后保持新高度，退出再下使能。双臂目标仍相对胸部，不做地面坐标补偿。

```bash
bash setup.sh all
bash run.sh pico check
bash run.sh pico install-apk
bash run.sh pico service
```

安装器在项目中准备 Python 环境、PC Service、APK、ADB 与 SDK 绑定；版本、URL、SHA-256 见 [assets.json](assets.json)。USB 用于首次安装，日常输入通过 Wi-Fi。

头显填写电脑地址，显示 WORKING，开启 Controller Tracking/Send，并关闭 A 键切换 Send。另一个终端：

```bash
bash run.sh pico monitor --duration 10
bash run.sh pico preview --sensitivity 0.25
bash run.sh errors
bash run.sh pico fairino --check
bash run.sh vr
```

monitor/preview 不连接机器人；preview 使用虚拟起点。开机有故障按 [故障排查](../../docs/故障排查.md) 单次清错，再复查归零。

`vr` 会使能并先回 Home，右手柄控制左臂、左手柄控制右臂。Grip 握住跟随、松开保持；A/X 冻结旋转，B/Y 冻结位移；左摇杆上下调灵敏度、按压暂停；右摇杆按压 Home，再按中止。

| 组件 | 文件/目录 |
|---|---|
| 服务、APK、监视和预览 | [pico.py](pico.py) |
| 遥操主循环 | [vr_client.py](fairino/vr_client.py) |
| TCP 插值与 IK | [vr_cartesian.py](fairino/vr_cartesian.py)、[vr_plan.py](fairino/vr_plan.py) |
| 机器人执行 | [vr_robot.py](fairino/vr_robot.py) |
| Home | [vr_home.py](fairino/vr_home.py)、[home_pose.json](fairino/home_pose.json) |
| 清错 | [clear_errors.py](fairino/clear_errors.py) |
| 接收和 SDK 绑定 | [pico4/](../../src/lerobot/teleoperators/pico4/) |
| 双手柄 | [bi_pico4/](../../src/lerobot/teleoperators/bi_pico4/) |
| 运动学 | [TCP 模块](../../apps/fairino_tcp_probe/) |
| 上半身生命周期 | [上半身模块](../../apps/fairino_upper_probe/) |

`vr` 为 450/500/100 Hz、12 ms 缓冲；`pico fairino` 默认 90/200/100 Hz。先 TCP 位移线性/姿态 SLERP 插值再 IK，机器人没有额外 30 ms 关节追赶段。详见 [遥操与延迟](../../docs/PICO遥操有线方案与延迟说明.md)。

日志位于 `.local/pico4/fairino-logs/`。模型限位、逆解、急停、故障及断流检查仍有效。当前未提供完整 LeRobot 数据集录制接口。

```bash
bash run.sh test
bash run.sh pico fairino --link-check --observe-ros \
  --duration 25 --interpolation-hz 450 --receiver-hz 500
bash run.sh pico latency --duration 25
```
