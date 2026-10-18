# 地图查看与按需建图

在仓库根目录运行，SSH 密码在终端输入：

```bash
bash run.sh map
```

默认查看明确选用的地图，底图保持不变；没有选用地图时显示“尚无底图”。右侧独立显示机器人周围 10 米范围的实时雷达；没有有效地图定位时，不把雷达或机器人箭头画到静态地图上。支持全图、跟随、缩放/平移和保存截图。截图与同一帧数据保存在 `logs/map-viewer/`，不会保存或覆盖机器人导航地图。

窗口提供明确的模式操作：

| 操作 | 行为 |
|---|---|
| 准备新建图 | 最多等待 90 秒有效的轮速、建图里程计、TF 与雷达输入，之后才启动新的 SLAM 会话 |
| 暂停建图 | 停止向工作地图加入新扫描，保留工作地图 |
| 继续建图 | 输入检查通过后继续更新工作地图 |
| 另存地图 | 先暂停，再把栅格地图和可继续建图的位姿图保存到新目录，禁止覆盖同名目录 |
| 结束建图 / 返回底图 | 先归档当前工作地图，再结束 SLAM；有默认地图则显示它，否则保持空底图 |
| 冻结显示 | 只冻结画面，不暂停后台建图 |

打开/关闭窗口本身不启动或停止建图。窗口不发送运动指令、不使能控制器。电脑无需 ROS，数据通过 SSH 读取。操作进行中需等待完成后关闭窗口。

推荐使用 `bash run.sh console` 的“建图与底盘”页：地图、双雷达和底盘键控同页显示，可边按键移动边看地图。独立运行时，`bash run.sh map` 与 `bash run.sh base` 仍可并排使用。切换到其他页面或窗口会暂停底盘移动并保持使能；切回后松键重按。空格 / Esc 或结束按钮才结束底盘控制并请求下使能。

启动底盘并确认里程计更新后，点击“准备新建图”；也可先点击准备，在 90 秒内启动底盘。默认原始模式监视厂家轮速、直通 `/odom`、TF 和雷达；断流、时间戳异常或守护程序因硬件故障停用时自动暂停，不自动恢复。原始模式不启动 EKF，也不使用 EKF 的位置方差阈值。暂停后界面顶部用红色提示“不再采集”，不会因仍能看到雷达刷新而误认为地图正在累计。输入检查不能替代运动精度验证。

2026-10-18 的建图修复使用独立 SLAM 程序和专用 `mapping.yaml`：后端按机器人平移约 10 厘米或转动约 5° 选帧，前端最多每 0.2 秒提交一次扫描，地图每秒发布。静止时不会反复累加关键帧。界面单独显示最近一次成功雷达匹配时间与本次连接观察到的匹配次数；运动已达到取帧阈值却超过 2 秒没有新匹配时，红色提示“当前 TF 主要随轮速推算”。静止时旧匹配显示为等待移动，不误报断流。只看地图消息或 TF 刷新不能判断匹配正常。

命令行入口与窗口操作一致：

```bash
bash run.sh mapping status
bash run.sh mapping check
bash run.sh mapping start --wait 90
bash run.sh mapping pause
bash run.sh mapping resume --wait 90
bash run.sh mapping save --name room_01
bash run.sh mapping view
bash run.sh mapping use --name room_01
```

新地图在机器人 `/home/fairino/maps/manual/<名称>/`，包括 `map.pgm`、`map.yaml`、`map.posegraph` 和 `map.data`。结束建图时自动生成 `archive-<时间>` 目录。保存新地图不会自动把它选为默认地图，需明确执行 `mapping use --name room_01`。

`bash run.sh mapping clear` 会停用地图发布，将 A/B 两套项目的 `maps/` 和 `/home/fairino/maps/` 移出地图库，再创建空目录。恢复备份保留在机器人 `/home/fairino/.local/share/fairino-map-control/cleared-libraries/<时间>/`，其中 `manifest.json` 记录原路径。2026-10-16 已按要求清空；恢复备份不会作为默认地图加载。

## 机器人部署与默认策略

`tools/install_map_mode.py` 会先备份，再将底盘启动脚本的 `AMR_NAV2_SLAM` 默认值改为 `false`，并安装 systemd 环境覆盖及手动建图服务。`fairino-manual-mapping.service` 没有开机启用项，只由明确的建图操作启动。工具不重启 `amr_startup.service`。

部署入口为 `.venv/bin/python tools/install_map_mode.py`；机器人端备份位于 `/home/fairino/.local/share/fairino-map-control/backups/`，本地部署记录位于 `logs/map-mode-change/installation.json`。随后 `bash run.sh mapping view` 归档已有 SLAM 会话并立即切回静态地图。不要通过重启整套底盘服务来应用当前模式切换。

扫描筛选修复的维护部署入口为 `.venv/bin/python tools/install_mapping_runtime.py --execute`，先编译和隔离验证，成功后再选择独立程序；不重启硬件，下一次新建图生效。实现与回退说明见 [独立建图修复](slam_runtime/README.md)。

短时检查连接并导出画面：

```bash
bash run.sh map --check --duration 12
```

可用 `--host`、`--user` 改 SSH 端点，`--output` 改截图目录，`MAP_PYTHON` 指定带 Tk、matplotlib、numpy 和 pexpect 的 Python。默认优先项目环境，缺 Tk 时尝试项目 PICO 环境。启动脚本会优先使用项目 `.local/map-gui/root/` 中支持 Xft 的 Tk；窗口使用 Noto Sans CJK SC，避免旧 Tk 的点阵字体缺字问题。

若 PICO 环境缺绘图库，可在仓库根目录执行 `.local/pico4/env/bin/python -m pip install -r requirements/diagnostics.txt`。本机已补齐并完成实时连接与图形显示检查。

## 开始新地图前

2026-10-16 发现的私有键控控制器与 EKF 输入不一致问题，已通过复用厂家 `eight_dof_chassis_controller` 和里程计守护程序处理；2026-10-17 重启后已确认守护程序运行。底盘结束后停止 EKF，不继续用旧速度预测。

2026-10-18 采用更简单的原始里程计模式：厂家 odom 的平面位姿、速度和原时间戳直接提供给 SLAM，不经过 EKF。坐标适配只接受已核对的 `base_footprint → base_link` 纯高度差，发布地面平面 TF；不额外计算位置或协方差。配置通过 `tools/install_control_stack.py --execute --odometry raw` 安装，守护程序重启后生效。界面显示“原始轮速直通，未使用 EKF”。原模式可用 `--odometry ekf` 恢复，其旧方差阈值仍保留；原问题见 [故障排查](../../docs/故障排查.md#建图很快因位置不确定度过大而暂停)。

先低速建一个小范围地图，检查静止时墙面是否稳定、直线/转向后是否出现重影，最后回到起点检查闭环。此前 1 米测试的 0.9916 米仅为编码器重算结果，不能代替完整位姿验证。

需要对照定位偏差时，可单独执行 `.venv/bin/python tools/record_odometry.py --duration 120`。它同时记录厂家原始 odom、建图 `/odom`、`map → odom` / `odom → base_footprint` TF、SLAM `/pose` 成功匹配以及键控指令；按一次 Ctrl+C 提前结束并保存。该采集器只读，不启动或停止控制。
