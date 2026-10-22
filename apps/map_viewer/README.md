# 地图查看、按需建图与定位

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

可用 `--host`、`--user` 改 SSH 端点，`--output` 改截图目录，`MAP_PYTHON` 指定 Python 环境。默认优先项目 `.venv`，独立窗口与工作台共用 GTK 3 地图面板，需要 GTK/Cairo、matplotlib、numpy 和 pexpect；可运行 `bash setup.sh console` 安装。`--check` 使用 Agg 离线绘图后端，不要求 GTK 或桌面显示。图中使用 Noto Sans CJK SC 显示中文。

建议在仓库根目录通过 `bash run.sh map` 或 `bash run.sh console` 启动，复用本机项目环境。复制仓库到另一台电脑后仍需安装对应依赖，历史连接记录不代表新电脑已完成联调。

## 开始新地图前

2026-10-16 发现的私有键控控制器与 EKF 输入不一致问题，已通过复用厂家 `eight_dof_chassis_controller` 和里程计守护程序处理；2026-10-17 重启后已确认守护程序运行。底盘结束后停止 EKF，不继续用旧速度预测。

2026-10-18 采用更简单的原始里程计模式：厂家 odom 的平面位姿、速度和原时间戳直接提供给 SLAM，不经过 EKF。坐标适配只接受已核对的 `base_footprint → base_link` 纯高度差，发布地面平面 TF；不额外计算位置或协方差。配置通过 `tools/install_control_stack.py --execute --odometry raw` 安装，守护程序重启后生效。界面显示“原始轮速直通，未使用 EKF”。原模式可用 `--odometry ekf` 恢复，其旧方差阈值仍保留；原问题见 [故障排查](../../docs/故障排查.md#建图很快因位置不确定度过大而暂停)。

先低速建一个小范围地图，检查静止时墙面是否稳定、直线/转向后是否出现重影，最后回到起点检查闭环。此前 1 米测试的 0.9916 米仅为编码器重算结果，不能代替完整位姿验证。

需要对照定位偏差时，可单独执行 `.venv/bin/python tools/record_odometry.py --duration 120`。它同时记录厂家原始 odom、建图 `/odom`、`map → odom` / `odom → base_footprint` TF、SLAM `/pose` 成功匹配以及键控指令；按一次 Ctrl+C 提前结束并保存。该采集器只读，不启动或停止控制。

## 已有地图定位（AMCL）

定位代码与建图代码同属本模块，共用工作台地图面板。默认仍为查看地图；打开窗口不会启动 AMCL。当前允许默认地图为空，程序不会自动选旧地图，也不会假定机器人在地图原点。

完成新地图后，在“建图与底盘”页依次操作：

1. **另存地图**，然后**结束建图 / 返回底图**。
2. **刷新地图库**，从列表明确选择刚保存的地图，点击**选用地图**。刷新列表本身不选择、不加载地图。
3. 确认底盘里程计和双雷达正在更新，点击**启动定位**。
4. 点击**设置初始位姿**。可以输入地图中的 X/Y（米）与朝向（度，地图 +X 为 0°，逆时针为正），或选择“地图拖拽”，在左侧地图从机器人当前位置拖向机器人正前方，核对数值后发送。
5. 等待本次初始位姿之后的雷达匹配。检查地图上墙体与实时雷达是否重合；随后低速前进、横移、转向并回到起点，检查位置、朝向和重合情况。初始位姿设置不产生运动。

命令行共用现有 `mapping` 入口，避免另建重复的地图控制链路：

```bash
bash run.sh mapping maps
bash run.sh mapping use --name room_01
bash run.sh mapping localize --wait 90
# 此处必须使用机器人在所选地图中的实际位置；不要直接照抄示例数值。
bash run.sh mapping initial-pose --x 1.2 --y -0.4 --yaw 1.5707963268
bash run.sh mapping status
bash run.sh mapping stop-localization
```

**CLI 的 yaw 单位是弧度，界面朝向单位是度。** 初始分布默认 XY 标准差 0.5 m、朝向标准差 15°。服务请求送达只说明 AMCL 收到请求，不代表匹配已正确。

定位输入为选用的静态地图、合并 `/scan` 和 `odom → base_footprint` TF；输出为 `/amcl_pose` 及 `map → odom`。专用参数在 [localization.yaml](localization.yaml)，使用全向模型 `nav2_amcl::OmniMotionModel`，适配底盘横移。定位不改变轮速里程计、不更新地图。机器人已有 AMCL 和 map_server 节点必须可用；只管理这两个节点的生命周期，不启动 Nav2 导航控制器。

[localization.py](localization.py) 管理定位会话和初始位姿；`map_control_remote.py` 对地图/定位修改加互斥锁。准备新建图、切换/清空地图库或返回底图会先停用 AMCL。定位启动拒绝正在运行的 SLAM，以免两个模块同时发布 `map → odom`。切换地图或重新启动定位都必须重新设置初始位姿。

`fairino-localization.service` 是显式启动的输入监视服务，持续检查轮速、直通/滤波里程计、TF、雷达及里程计会话。输入持续异常约 1.5 秒或里程计会话变化后停用定位，不自动恢复。会话状态保存在机器人 `~/.local/share/fairino-map-control/localization.json`，不是地图数据。关闭工作台窗口不停止定位；需要时显式点击“停止定位”。

界面只接受本次地图/初始位姿会话的 AMCL 估计，并检查活动节点、监视服务心跳、输入新鲜度、TF 和协方差。静止时允许使用本次会话的历史估计；移动累计约 10 cm 或 5° 后超过 2 秒没有新估计会标记不可用。位置最大主轴标准差超过 0.5 m、朝向标准差超过 20° 时提示不确定度较大。这些是初始监视阈值，不能证明真实定位正确，仍需检查雷达与地图重合。监视异常时不在静态地图上显示机器人和映射后的雷达，右侧本体雷达仍可查看。

### 文件、部署与验证范围

- 定位业务及参数：`apps/map_viewer/localization.py`、`localization.yaml`。
- GUI：共用 `apps/map_viewer/panel.py`；遥测：`remote_stream.py`。
- 运维入口：`tools/map_mode.py`、`tools/install_map_mode.py`。
- 后续导航轮廓接口：[`apps/navigation/footprint.py`](../navigation/footprint.py)，固定底盘轮廓已实现，上半身动态投影仍预留；详见 [二维导航说明](../navigation/README.md)。
- 地图仍保存到机器人 `/home/fairino/maps/manual/<名称>/`，日志保存到仓库 `logs/`；不把地图、运行状态或生成文件放到源码目录。

部署沿用 `tools/install_map_mode.py`：备份并安装定位代码、参数、监视服务，执行 systemd daemon-reload，不重启底盘、不自动启动定位。已有安装需要更新，否则工作台新按钮会报告机器人端尚不支持。仅更新 `install_control_stack.py` 不会创建定位监视服务，完整定位部署仍需运行地图安装器。

2026-10-22 从另一台控制电脑同步定位与导航集成代码，地图模块 42 项离线测试通过，并在本机项目环境中用假数据完成 GTK 地图/定位/导航面板实例化与按钮状态检查（未连接机器人）。远端保存的 2026-10-20 记录显示定位与导航代码已部署到机器人，并核对了文件哈希；这些是历史部署记录，不能据此推断当前服务状态或定位精度。本次代码同步不执行机器人部署、不加载地图、不启动定位或运动；实际定位精度仍需用当前地图和雷达重合、直线/转向及回到起点验证。

实现依据：[Nav2 Humble AMCL 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp)。使用标准 `/set_initial_pose` 服务，兼容 AMCL 的 `/initialpose` 话题；重启后重新初始化，不沿用旧会话位姿。

地图下方新增导航栏：接入导航底盘、启动导航、输入目标、预览路径、开始/取消目标及结束下使能。输入目标后显示红色 ❌ 和目标朝向，尚未点击“开始导航”时不会移动。地图或定位会话变化会清除目标，具体步骤见 [工作台导航操作](../navigation/README.md#工作台操作)。


## 初始位姿后的定位估计

拖拽设置的是大致搜索起点和朝向，不是最终位置。2026-10-20 起，定位监视服务在输入健康且机器人静止时，对新雷达数据额外请求最多 6 次 AMCL 更新（间隔至少 1 秒，窗口最多 12 秒），随后回到常规运动触发更新。不会为定位自动移动或旋转底盘。请求失败保留已发送的初始位姿并显示失败原因，不再将已接受的初始化一并抹掉。

初始化期间和不确定度较大时，地图显示橙色候选机器人箭头及雷达投影；这是待核对的估计，不代表可以导航。界面同时显示 AMCL 估计的 X/Y/朝向、本次观察到的位姿更新次数。位置/朝向标准差表示估计不确定度，并不是与真实位置的实测误差。定位、TF、里程计或雷达失效时仍隐藏候选位置；初始化期间导航保持禁用。

静止更新次数完成不等于位置正确，也不等于全局定位。默认初始分布仍为位置 σ=0.5 m、朝向 σ=15°；大致位姿偏差过大、地图不一致或场景重复时可能无法定位到正确位置。需核对雷达与地图墙面重合，再开始导航。当前全局定位按钮仍未实现。

对应代码为 localization.py（有限静止更新）、remote_stream.py（候选显示与导航就绪分离）、panel.py/viewer.py（坐标、不确定度和橙色候选箭头）。新界面与观察进程需重开工作台加载，已有控制会话不会热替换。
