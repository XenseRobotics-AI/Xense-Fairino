# 双臂 TCP 连续键控

模块 `apps/fairino_tcp_probe/`。仓库根目录执行：

```bash
bash run.sh tcp --arm left
bash run.sh tcp --arm right
```

连接上半身 `192.168.58.1`，使能全部 J1–J20 并保持当前姿态；界面可选臂，长按连续运动、松键减速停止。

| 分量 | 正向 | 负向 |
|---|---|---|
| X 前/后 | W | S |
| Y 左/右 | A | D |
| Z 上/下 | R | F |
| 绕胸部 X | I | K |
| 绕胸部 Y | J | L |
| 绕胸部 Z | U | O |

统一胸部 X 前、Y 左、Z 上，随胸部姿态变化。TCP 为法兰中心，腕心偏移 `[-106,0,0] mm`。空格或失焦停止并保持；Esc/关闭及故障结束并请求下使能。

关节计划速度默认 2°/s，范围 0.1–4°/s；TCP 平移默认/上限 8 mm/s，旋转默认/上限 2°/s。该入口以局部 IK 求关节速度，再由机器人端短轨迹执行，与 PICO 的插值位置控制分开实现。

`tcp_keyboard.py` 为界面；`tcp_core.py`、`tcp_frames.py` 为模型与胸部变换；`tcp_jog.py` 为速度/减速规划；`tcp_robot.py` 为 ROS 后端。`vendor_kinematics.py` 加载项目内 `vendor/fairino/` 数学库，模型位于 `models/`。

```bash
bash run.sh tcp --demo
bash run.sh tcp --check
bash run.sh tcp --prepare-check
bash run.sh diagnostics wrist
bash run.sh test
```

模型校验见 [analysis/](analysis/README.md)，日志生成于 `apps/fairino_tcp_probe/logs/`。
