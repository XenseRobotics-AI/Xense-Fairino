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

普通启动复用 `eight_dof_chassis_controller`（尚未加载时以同名加载），原始里程计固定发布到 `/eight_dof_chassis_controller/odom`；不再创建 `fairino_keyboard_chassis`。速度输入仍隔离在 `/fairino_keyboard/cmd_vel`。退出先发送零速并下使能；若本次控制已完成启动、下使能获确认且硬件反馈健康，保持已有厂家实例活动，使原始轮速与 /odom 继续测量，不恢复或重置里程计会话。启动失败、硬件故障、反馈异常或下使能未确认时仍暂停里程计并停用厂家实例。临时转向回正控制器仍卸载。`--check` 只读；`--prepare-check` 验证接口后停用，不使能、不回正。首次使用需按根目录 README 迁移配置。

故障退出时若厂家拒绝下使能，临时控制器会保留，日志分别报告下使能和清理结果。完成清错并确认全部下使能后，下次点击启动会自动检查并回收 `inactive` / `unconfigured` 的 `fairino_keyboard_angle` 和旧 `fairino_keyboard_chassis`。恢复持有与键控、PICO 底盘及底盘清错相同的控制锁；每次卸载前等待新的底盘和上身急停反馈，要求无故障、输入回中、全部下使能、实测速度绝对值不超过 0.03 rad/s、速度指令绝对值不超过 0.005 rad/s。仍活动的临时控制器不会被强行停用或卸载。卸载后重新查询确认对象消失，才进入原有回正及使能流程；厂家底盘实例保留。`--check` 不执行回收。

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

2026-10-20：修正控制会话退出无条件中断里程计的问题，新增 5 项离线退出流程回归。此修正不会放宽通信或硬件超时保护；实机连续里程计验证尚待完成。Worker 随底盘连接发送，重新打开工作台并建立新的底盘会话才会加载更新，已有会话不会被热替换。

## SSH 会话优先级与往返通信超时

2026-10-20 在机器人上发现厂家 limits.conf 将 realtime 组的普通 priority 写为 60，导致 fairino 的 SSH 会话及控制进程实际为 nice 19。priority 是普通进程 nice 值，rtprio 才是实时调度权限；两者不能混用。依据：[Linux-PAM limits.conf](https://man7.org/linux/man-pages/man5/limits.conf.5.html)。

已通过 tools/install_session_priority_remote.py 在机器人新增 /etc/security/limits.d/90-fairino-session-priority.conf，仅覆盖 fairino 用户的普通 priority 为 0，保留厂家 rtprio、memlock 设置。脚本默认只打印配置，--execute 才写入；修改前备份，使用新 PAM 登录验证 nice 0，失败回滚。现有 SSH 会话不自动改变优先级，重开工作台并重新连接控制会话后生效。

tools/check_control_link.py 是不使能电机的诊断入口，默认 60 秒，复用工作台 SSH 传输、机器人反馈订阅和输入 ACK 检查。它替换 Worker 的运行入口，只测量而不调用 startup，也不获取运动所有权或发布速度；仍持有同一独占锁，不能与实际底盘控制并行。需要已有共享 SSH 连接及机器人 sudo 权限。--nice 19 可仅在测试进程中模拟旧优先级，默认 --nice 0 使用普通优先级。

两次同类只读测试中，nice 19 的 ACK 最大年龄为 531 ms，并出现超时；nice 0 连续 60 秒最大为 176 ms，未出现超时。这里测量的是应用层确认的年龄，不是 ping 的网络延迟。完整工作台使能及导航仍需验证。运行记录位于 logs/navigation-heartbeat-20261020/。

状态日志增加 link.input_age、ack_age、client_status_age、client_send_gap、process_nice；超时报错包含实际 ACK 年龄。原有 450 ms 通信保护与 400 ms 硬件反馈保护均保持不变，断流仍停止并下使能，不自动重试或恢复运动。
