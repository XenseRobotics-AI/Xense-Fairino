# 四舵轮底盘键控

模块 `apps/fairino_keyboard/`。仓库根目录执行：

```bash
bash run.sh base --check
bash run.sh base --demo
bash run.sh base
```

通过 SSH 连接底盘 `192.168.58.3`，ROS Domain 1。普通启动回正四个转向轴后使能八轴。

| 按键 | 行为 |
|---|---|
| W/S | 前进/后退 |
| A/D | 横移 |
| J/L | 自转 |
| 组合键 | 平移/旋转叠加；斜向归一化 |
| 松键 | 零速度、减速停止 |
| 空格/Esc、失焦、关闭 | 结束并请求下使能 |

默认平移 0.06 m/s、自转 0.10 rad/s；界面范围 0.01–0.20 m/s、0.01–0.40 rad/s。方向以机器人自身为准。

```bash
bash run.sh base --linear 0.10 --angular 0.20
bash run.sh base --prepare-check
bash run.sh test
```

`keyboard_control.py` 为界面/SSH，`robot_worker.py` 为机器人后端，`control_core.py` 为纯逻辑。回正将 J1/J3/J5/J7 移到现有标定的 0 rad，不修改编码器零点。故障、急停、反馈/心跳及异常轮速检查有效。日志生成于 `apps/fairino_keyboard/logs/`。

开机错误先按 [故障排查](../../docs/故障排查.md) 处理。
