# 双臂遥操与延迟

全部命令在仓库根目录执行。头显填写电脑局域网地址，开启 Controller Tracking 和 Send，关闭 A 键切换 Send。

```bash
bash run.sh pico service
# 另一个终端先检查数据，再退出 monitor：
bash run.sh pico monitor --duration 10
bash run.sh vr
```

自定义命令：

```bash
bash run.sh pico fairino --execute \
  --mode mirror_same_direction --rotation-mode mirror --hand-mapping crossed \
  --freeze-buttons --sensitivity 0.25 \
  --interpolation-hz 450 --receiver-hz 500 --interpolation-delay-ms 12
```

PICO → Wi-Fi → 电脑相对映射/灵敏度/冻结 → TCP 位移线性插值与四元数 SLERP → 厂家 IK → 有线 SSH → 机器人 ROS 单点位置保持 → EtherCAT 驱动器。

`vr` 指定 450 Hz 插值/IK/下发、500 Hz 接收轮询、100 Hz 状态回传、12 ms 输入缓冲。`pico fairino` 默认 90/200/100 Hz。要求下发≤接收≤500 Hz；实际频率看日志。缺少未来样本时保持最近目标，不外推；机器人没有额外 30 ms 关节追赶段。

胸部坐标 X 前、Y 左、Z 上，随胸部姿态变化。面对面同房间方向平移，独立镜像旋转；右手柄→左臂，左手柄→右臂。Grip 握住跟随、松开保持；A/X 冻结旋转，B/Y 冻结位移；左摇杆上下调灵敏度、按压暂停；右摇杆按压 Home，再按中止。默认启动先回 Home，`--no-auto-home` 可跳过。

## 延迟测量

```bash
bash run.sh pico fairino --link-check --observe-ros \
  --duration 25 --interpolation-hz 450 --receiver-hz 500
bash run.sh pico latency --duration 25
```

link-check 不使能、不发布运动目标；latency 需要头显 Send 和 USB/ADB 校时。结果写入 `.local/pico4/fairino-logs/`。

已有 450 Hz 测量中，无线有效处理约 230 Hz，有线约 438 Hz；有线实机稳定段下发约 449.5 Hz、ROS 接收发布约 433.1 Hz。每次运行的负载和网络不同，应重新测量。

输入传输、12 ms 缓冲、IK、目标传输、机器人处理与电机响应属于不同阶段。样本年龄不等于网络单程延迟，反馈确认不等于物理到位。P95 表示 95% 的样本不超过该值；各段 P95 不能直接相加为总延迟。

代码：[电脑主循环](../scripts/pico4/fairino/vr_client.py)、[TCP 插值](../scripts/pico4/fairino/vr_cartesian.py)、[IK](../scripts/pico4/fairino/vr_plan.py)、[机器人执行](../scripts/pico4/fairino/vr_robot.py)、[PICO 计时](../scripts/pico4/fairino/pico_latency.py)。
