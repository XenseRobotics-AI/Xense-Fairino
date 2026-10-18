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
| 切换页面 / 窗口 | 暂停移动、保持使能；切回后松键重按 |
| 空格/Esc、关闭 | 结束并请求下使能 |

默认平移 0.06 m/s、自转 0.10 rad/s；界面范围 0.01–0.20 m/s、0.01–0.40 rad/s。方向以机器人自身为准。

普通启动复用 `eight_dof_chassis_controller`（尚未加载时以同名加载），原始里程计固定发布到 `/eight_dof_chassis_controller/odom`；不再创建 `fairino_keyboard_chassis`。速度输入仍隔离在 `/fairino_keyboard/cmd_vel`。退出停用厂家实例并暂停 EKF，保留已加载实例；临时转向回正控制器仍卸载。`--check` 只读；`--prepare-check` 验证接口后停用，不使能、不回正。首次使用需按根目录 README 迁移配置。

测试时先在第一个终端启动只读采集，输入 SSH 密码后，再在第二个终端启动键控。移动过程中保持键控窗口焦点；切窗会暂停移动并保持使能，切回后松键重按继续。录制 120 秒，为键控启动、回正和人工测试留出时间：

```bash
.venv/bin/python tools/record_odometry.py --duration 120
# 若同一采样区间内人工实测总路程为 1 米，可计算路程误差：
.venv/bin/python tools/record_odometry.py --duration 120 --reference-distance-m 1.0
```

出现“采集已开始”后，工具每 5 秒显示进度，并持续写入 `events.jsonl`。测试完成可按一次 `Ctrl+C` 提前结束，已收到的数据、汇总和图表仍会保存。

记录包括统一轮速里程计、滤波 `/odom`、键控速度指令和底盘状态，JSON 与轨迹/速度图保存到 `logs/*-odometry-test/`。采集工具兼容观察旧 `/fairino_keyboard/odom` 便于对比，但新控制路径不再发布该话题。工具只订阅和查询，不激活控制器。

```bash
bash run.sh base --linear 0.10 --angular 0.20
bash run.sh base --prepare-check
bash run.sh test
```

`keyboard_control.py` 为界面/SSH，`robot_worker.py` 为机器人后端，`control_core.py` 为纯逻辑。回正将 J1/J3/J5/J7 移到现有标定的 0 rad，不修改编码器零点。故障、急停、反馈/心跳及异常轮速检查有效。日志生成于 `apps/fairino_keyboard/logs/`。

开机错误先按 [故障排查](../../docs/故障排查.md) 处理。
