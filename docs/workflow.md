# 建图到导航的完整操作

本页为后续现场操作说明，本次迁移没有连接机器人。

## 连接和故障检查

控制电脑的机器人接口需有未占用的 `192.168.58.x/24` 地址。Linux 可用 `ip route get 192.168.58.3` 查看实际接口与源地址；不要让机器人有线网段接管默认互联网路由。

```bash
bash scripts/run.sh network
bash scripts/run.sh faults
bash scripts/run.sh inspect
bash scripts/run.sh chassis --check
```

遇到遗留控制器、清错后仍无法接入或下使能未确认，可使用地图窗口的 **恢复底盘接入**。建图中先保存地图；恢复成功后手动重新接入、重新初始化定位，再发送新目标。该操作会结束本窗口的底盘控制并暂停里程计，不自动使能或继续旧目标。命令行可用 `bash scripts/run.sh recover` 只读检查，`bash scripts/run.sh recover --execute` 显式执行一次。

故障清除及恢复的限制见 [故障处理](troubleshooting.md)。`faults --execute` 是明确的单次复位/下使能操作；不是只读检查。

## 建图

```bash
bash scripts/run.sh map --host 192.168.58.3 --user fairino
```

1. 地图窗口先建立只读观察连接。
2. 点击启动建图底盘，等待四个转向轴回正和八轴使能。
3. 等待厂家原始 odom、建图 `/odom` 和雷达更新，再点击准备新建图。
4. 在建图控制区按住 W/S 前后、A/D 横移、J/L 自转；松键停止。调整速度前松开运动键。
5. 暂停建图，另存到新名称，再结束建图返回底图。冻结显示只冻结画面。
6. 结束建图底盘，等待下使能确认。关闭窗口也需要完成底盘退出流程。

地图管理也可从另一终端操作，底盘仍由建图窗口持有：

```bash
bash scripts/run.sh mapping start --wait 90
bash scripts/run.sh mapping pause
bash scripts/run.sh mapping save --name room_a
bash scripts/run.sh mapping view
bash scripts/run.sh mapping maps
bash scripts/run.sh mapping use --name room_a
```

`mapping resume` 仅对同一里程计会话有效。里程计已重建时，应保存/结束旧地图并开始新会话。

## 已有地图定位和导航

GUI 顺序：结束建图键控 → 选用地图 → 接入导航底盘 → 启动定位 → 设置初始位姿 → 确认定位就绪 → 启动导航 → 输入目标 → 可选预览路径 → 开始导航。

CLI 等价流程，在一个终端保持底盘执行连接：

```bash
bash scripts/run.sh navigation drive
```

另一个终端完成定位和导航：

```bash
bash scripts/run.sh mapping use --name room_a
bash scripts/run.sh mapping localize --wait 90
bash scripts/run.sh mapping initial-pose --x 0 --y 0 --yaw 0
bash scripts/run.sh navigation status
bash scripts/run.sh navigation start
bash scripts/run.sh navigation plan --x 1 --y 0
bash scripts/run.sh navigation goal --x 1 --y 0
```

坐标仅为格式示例，必须替换为现场地图中的有效位置。CLI yaw 单位弧度，GUI 朝向输入单位度。定位不确定度是估计值，不代表实测误差；仍需核对扫描和墙面重合。

```bash
bash scripts/run.sh navigation cancel
bash scripts/run.sh navigation stop
```

最后在 `navigation drive` 终端 Ctrl+C，等待下使能确认。取消目标本身不下使能。

## 诊断

```bash
bash scripts/run.sh record-odometry --duration 120
bash scripts/run.sh check-link --help
bash scripts/run.sh check-controller --help
```

里程计记录只读。链路检查复用真实 SSH/ROS 传输与独占锁，需要机器人连接，但不调用回正/使能入口。`check-controller` 是需要 ROS/Nav2 的本机隔离 Domain 87 测试，不是普通电脑的纯 Python 离线测试；本次未运行它。

导航目标使用 map 绝对坐标。默认 `hold_initial` 在发送任务时按新鲜定位锁定朝向，预览不会锁定后续任务，旧 `--yaw` 会被忽略。若将配置中的 `FollowPath.heading_mode` 改为 `path_tangent` 并重新部署导航、重启面板，则沿途朝向跟随路径方向，终点朝向由用户输入：GUI 使用度，CLI 需要 `--yaw` 弧度值（例如 90° 为 `1.5707963267948966`）。两种模式都不会因填写坐标而自动移动。详见 [预瞄跟随与朝向策略](navigation.md)。
