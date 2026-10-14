# PICO 接收与双臂遥操

模块位于 `scripts/pico4/`。全部命令从仓库根目录运行。

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
