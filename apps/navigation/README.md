# 二维底盘导航

本模块使用 Nav2 Humble：**SmacPlanner2D（二维 A*）+ DWB 全向局部控制**。仅考虑底盘在 XY 平面中的碰撞和运动。上半身投影提供器仍为预留接口，不参与当前规划。

## 固定底盘轮廓

模型来源是 2026-10-19 从机器人运行配置保存的 [amr_8dof.urdf](../../models/amr_8dof.urdf)，原路径及坐标约定见 [雷达与模型](../../docs/雷达与模型.md)。这里不使用坐标约定不同的历史整机 CAD URDF。

`ChassisFootprint` 从 `base_link/collision` 的 box 读取尺寸和 origin，将 8 个顶点转换后投影到 XY 平面，求凸包。当前底盘尺寸 **0.750 × 0.600 m**，面积 **0.450 m²**，原点为底盘中心：

```text
[(-0.375, -0.300), (0.375, -0.300), (0.375, 0.300), (-0.375, 0.300)]
```

顶点按逆时针排列，单位米，X 向前、Y 向左。URDF 中的高度不影响二维投影。当前只实现运行版底盘碰撞盒；遇到未支持的 mesh 等几何时明确报错，不会忽略碰撞体。模型校验值记录在 `models/manifest.json`。

接口保持可选择：

```python
from apps.navigation.footprint import selector, FootprintRequest
provider = selector("models/amr_8dof.urdf")
shape = provider.compute("chassis", FootprintRequest(stamp=0.0, joint_positions={}))
```

`upper_body_projection` 尚未实现，选择它会报错，不会自动退回较小的底盘轮廓。后续动态提供器须根据实际关节反馈和碰撞模型计算包络，并检查反馈时间。

## 二维规划和速度

`profile.py` 生成 `nav2.yaml`，参数统一来自上述 URDF 与 `apps/fairino_keyboard/control_core.py` 中的默认速度：

| 项目 | 当前值 |
|---|---|
| 平移合速度 | ≤ 0.06 m/s，包括前后、横移和斜向移动 |
| 旋转速度 | ≤ 0.10 rad/s |
| 局部控制 | DWB，正负 X/Y 速度采样，ObstacleFootprint 多边形碰撞检查 |
| 目标容差 | 位置 0.05 m、朝向 0.05 rad |
| 地图分辨率 | 0.05 m |
| 局部代价地图 | 4 × 4 m 滚动窗口 |
| 轮廓余量 | 局部矩形每侧增加 0.02 m |
| 重规划 | 1 Hz |
| DWB 控制输出 | 10 Hz |

平移速度限制的是 `sqrt(vx² + vy²)`，不是只限制每个轴。配置、速度门控和底盘执行层都应用限制；不沿用键控允许调节的最大值 0.20 m/s / 0.40 rad/s。

SmacPlanner2D 不做随 yaw 变化的矩形碰撞搜索，而是检查膨胀代价。全局代价地图因此使用一个保守辅助轮廓：其内切圆半径等于加余量底盘的外接圆半径，约 **0.508 m**。这保留了矩形四角的空间要求，会比按实际朝向挤过窄通道更保守。局部 DWB 使用真实矩形加 2 cm 余量。辅助轮廓不用于清除车身附近障碍；它不是第二种上半身动态模式。

全局和局部地图均使用静态地图、二维 ObstacleLayer 和 InflationLayer。不使用三维 voxel 层。障碍层分别订阅 `/front/scan`、`/back/scan`，使清障射线从各雷达真实原点出发；定位继续使用合并 `/scan`。未知区域不可通行。当前关闭 Smac 附加平滑，以避免 Humble 平滑器对未知单元的放行行为。行为树只做重规划与跟随，不执行自动旋转/倒车恢复动作。

参考 [Humble Smac2D 实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_smac_planner/src/smac_planner_2d.cpp) 和 [DWB 轮廓碰撞检查](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp)。

## 代码入口与部署

所有命令从控制电脑的仓库根目录运行，使用本项目的 Python 环境：

```bash
.venv/bin/python tools/build_navigation.py --check
# URDF 或默认速度变化后，重新生成，再检查版本差异：
.venv/bin/python tools/build_navigation.py
```

连接机器人部署时，先结束底盘/PICO 控制和建图、定位、导航会话，确认底盘下使能，再更新地图定位层和导航：

```bash
.venv/bin/python tools/install_map_mode.py  # 此旧入口运行即备份并安装，没有预览模式
.venv/bin/python tools/install_navigation.py
.venv/bin/python tools/install_navigation.py --execute
```

只有 `install_navigation.py` 不带 `--execute` 时是机器人端只读前置检查；`install_map_mode.py` 运行即安装，也不支持 `--help`。导航安装器核对实际底盘轮廓与仓库模型一致，并检查 Nav2 插件是否存在；备份后安装到机器人 `~/.local/share/fairino-navigation/`，安装显式启动的 `fairino-navigation.service`。不自动启动服务、不改地图、不使能电机。服务运行中拒绝覆盖代码，先 `navigation stop` 再更新。启动文件、ROS 参数和行为树均位于 `apps/navigation/`，SSH/安装入口位于 `tools/`，运行记录位于仓库 `logs/navigation/`。已有统一底盘与原始里程计配置无需重新迁移；未安装过的机器先按根目录 README 完成控制栈迁移。

## 工作台操作

启动 `bash run.sh console`，在「建图与底盘」页连接只读状态。导航操作已集成在地图下方，不再需要单独保持执行终端：

1. 明确选用保存的地图，结束原来的手动底盘控制。
2. 点击 **① 接入导航底盘 · 回正使能**，先等待导航节点创建、配置为待机，再等待底盘回正使能就绪。准备失败不会创建底盘控制连接。该按钮复用原底盘回正、使能、反馈保护与独占锁；手动键控和导航不能同时占用底盘。
3. 使用上方已有的 **启动定位 → 设置初始位姿**，核对雷达与地图重合；然后点击 **② 启动导航**，激活已配置的节点，等待导航就绪。此步骤不启动新进程或创建新的 ROS 通信节点。
4. 在导航栏填写目标 **X / m、Y / m、朝向 / °**。这是 `map` 坐标系下的绝对位置；0° 沿地图 +X，90° 沿 +Y。输入完整有效值后，地图立即显示**红色 ❌**（红叉图元），旁边标出坐标和目标状态，小箭头表示目标朝向。输入本身不移动机器人。
5. 可先点击 **预览路径**，地图显示紫色虚线路径；点击 **③ 开始导航**才发送目标。预览是当次规划结果，执行中 Nav2 会重规划，并不将此虚线冒充实时执行路径。
6. **取消目标**取消导航、保持底盘使能；**结束导航并下使能**停止 Nav2 并结束底盘执行会话。关闭工作台同样等待导航停止和底盘退出，下使能结果显示在导航状态中。

成功、取消或失败后保留红叉并更新状态，便于核对终点；修改目标后变为“待发送”并清除旧预览。切换地图或定位会话后，清除旧目标和路径，必须重新输入。发送中的目标不能被编辑；取消后等待终态再输入新目标。请求超时会提示核对状态，不自动重发。

上面三个编号按钮分别对应原先终端的 `drive`、`start`、`goal`，定位仍使用同页已有控件。打开工作台不会自动使能、启动导航或发送目标。导航执行不依赖 W/A/S/D，切换标签页不取消已明确下发的目标；停止请使用取消/结束按钮。界面心跳中断、底盘反馈断流会触发原有停止保护。

导航面板与状态逻辑位于 `apps/navigation/panel.py`、`desktop.py`；地图红叉与路径由 `apps/map_viewer/viewer.py` 绘制，底盘占用仍由工作台统一管理。机器人端需更新导航部署以支持定位会话校验。此前版本有部署与下使能状态下的启动检查记录；2026-10-20 的连续执行参数修改仍待部署、验证。本地同步不会自动更新机器人，完整定位、行驶和避障仍需联调。

## 命令行使用顺序（保留）

默认地图可以为空；本轮没有创建、选用或加载任何地图。没有地图时 `navigation start` 会明确拒绝。

1. 完成建图、另存、结束建图，明确选用新地图。关闭之前的底盘键控会话。
2. 在一个终端运行 `bash run.sh navigation drive`，保持该终端运行。**该命令会按现有底盘流程回正、使能并接入导航执行**，但不自动发送导航目标。它与底盘键控共用独占锁，不能同时运行。
3. 在地图工作台启动定位并设置初始位姿，检查雷达与地图墙面重合。也可用 `mapping localize` 和 `mapping initial-pose`。
4. 在另一个终端启动 Nav2，等待定位和控制器就绪，再先计算路径：

   ```bash
   bash run.sh navigation start
   bash run.sh navigation status
   # X/Y 是 map 坐标系中的米，yaw 是弧度；替换为地图中实际可达目标。
   bash run.sh navigation plan --x 1.0 --y 0.0 --yaw 0.0
   ```

   `plan` 只调用 ComputePathToPose，不移动机器人，路径保存到 `logs/navigation/plan-*.json`。
5. 明确发送目标：

   ```bash
   bash run.sh navigation goal --x 1.0 --y 0.0 --yaw 0.0
   bash run.sh navigation status
   bash run.sh navigation cancel
   ```

   `goal` 调用 NavigateToPose；已有执行目标时要求先取消。状态里的 `last_goal.status` 使用标准 ROS action 状态：2 执行、4 成功、5 取消、6 失败。
6. 结束时先取消目标，再 `navigation stop`；在 `drive` 终端按 Ctrl+C，等待底盘已确认下使能的反馈。

导航节点位于独立 `/fairino_navigation` 命名空间，沿用已有地图、里程计和 TF，不另起 AMCL 或 SLAM：

```text
Nav2 controller → /fairino_navigation/cmd_vel_raw
                → velocity_gate（定位、目标、断流、限速）
                → /fairino_navigation/cmd_vel
                → 已明确启动的底盘 Worker（反馈、心跳、独占锁、再次限速）
                → /fairino_keyboard/cmd_vel → 厂家底盘控制器
```

`start` 只启动规划相关服务；真正执行还要求显式的 `drive` 会话和 `goal`。没有目标时速度为零。定位/里程计会话变化、双雷达或速度输入断流会停止输出，失败目标不会在数据恢复后自动继续。取消目标不等于下使能；Ctrl+C 结束 `drive` 才走底盘停止与下使能流程。

工作台和命令行共用导航操作接口。Nav2 已部署并做过服务启动检查；尚未验证实际行驶、避障及到点精度。

离线验证记录：导航/轮廓 15 项、地图/定位 33 项、底盘 27 项通过，4 项 GTK/显示测试跳过；生成配置一致性、Python/Shell 语法及补丁格式检查通过。完整记录保存于 `logs/navigation-20261019/verification.json`。

本次工作台集成新增 12 项离线回归，覆盖目标坐标与会话绑定、红叉/朝向绘制、底盘互斥、界面心跳停止、异步状态过期与目标取消。GTK 完整窗口交互仍待图形环境及真机联调。

## 导航启动与反馈连续性

普通“启动底盘”接入的是手动键控，不能替代“① 接入导航底盘”。工作台只有在导航底盘已就绪、地图反馈新鲜且定位就绪时才开放“② 启动导航”。若此前使用手动底盘，应先结束手动控制，再接入导航底盘，随后重新启动定位并设置初始位姿；底盘回正可能重新建立里程计会话。导航期间保持定位开启。

2026-10-20 曾测得集中启动导航进程导致硬件反馈间隔 0.751 秒。分批启动和降低进程优先级在一次下使能测试中降低了间隔，但随后完整运行仍出现上身急停反馈超时，因此不能视为已解决。现改为两阶段启动：

- ① 在获取底盘执行连接之前调用 prepare：持有与底盘 Worker 相同的独占锁，启动服务并配置三个 Nav2 节点到 inactive；确认待机后才允许回正、使能。服务 autostart=false，不自动激活。首次准备通常需要十几到几十秒；失败或取消不接入底盘。
- ② 只提交绑定运行实例、定位会话和里程计 epoch 的有限期激活请求。常驻 velocity_gate 使用启动时已创建的生命周期客户端执行 RESUME，不在使能路径创建新 ROS 节点或进程。过期请求、旧实例、定位未就绪及已有目标均拒绝。
- 导航就绪还要求三个节点实际处于 active 且状态新鲜；仅存在速度发布器不再视为控制器已就绪。
- 底盘原有 0.4 秒反馈保护保持不变。保护退出后界面保留具体错误，并显示下使能确认结果；不会自动使能、恢复目标或解除里程计断流锁定。

准备阶段的服务创建仍每隔 3 秒启动一个进程，并通过 nice -n 10 降低调度优先级。运行接口、状态机和安装文件位于 apps/navigation/；SSH 入口位于 tools/navigation_mode.py 和 tools/install_navigation.py。实机准备检查与离线回归记录见 logs/navigation-preparation-20261020/。完整带定位的激活与行驶仍需实机验证。

2026-10-20 后续心跳排查发现 fairino SSH 会话继承了错误的 nice 19。已通过用户级 PAM priority=0 覆盖修正，未放宽保护阈值。正常优先级下 60 秒只读心跳测试未超时，导航配置待机检查通过；详细安装、对照测试和日志字段见 apps/fairino_keyboard/README.md 的“SSH 会话优先级与往返通信超时”。重开工作台并建立新控制会话后生效。

## 连续执行与车轮转向（2026-10-20，待机器人验证）

正常运行时只需发送一次目标，控制器持续输出速度，直到到达、取消或保护停止。每走一小段就需要再次点击不属于正常操作。现场日志记录了连续多次目标取消，以及控制循环超过 0.3 秒的间隔；此前取消后的状态会覆盖首个原因，因此尚不能把每次取消都确认为速度断流。

本次代码调整：

- 控制频率改为 10 Hz，DWB 速度采样由 11×11×15 降到 5×5×7，局部代价地图更新为 5 Hz，行为树 tick 为 50 ms，以减轻计算负载。碰撞预测仍为 3 秒，保留底盘多边形碰撞检查、双雷达及原有安全阈值。
- 使用 Humble DWB 的 LimitedAccelGenerator，sim_period=0.1 秒，与控制周期一致。候选速度按当前里程计速度和加减速度约束生成，再参与轨迹碰撞评估，减少一次指令的大幅变化。平移最高速度仍为 0.06 m/s，角速度仍为 0.1 rad/s，保持全向二维移动。
- 导航与键控最终都进入同一厂家底盘控制器。导航可连续调整 x/y/yaw 组合，键控通常保持固定组合，两者轮子动作不必完全相同。此次配置未建立显式舵轮角度/转向速度模型，不能仅凭离线测试保证实车无摆动。
- 首次保护停止记录目标 ID、具体原因、速度指令年龄、传感器年龄、定位和生命周期状态到机器人服务日志；status.json 保留 failure。工作台在收到匹配目标的取消/失败终态后显示“保护停止”及原因；取消确认前继续锁定目标编辑。
- 速度断流 0.3 秒仍停止并锁定该目标，不自动重发或恢复。新目标必须由操作者明确提交。

控制电脑代码与离线测试完成，机器人连接中断，**此轮修改尚未部署到机器人**。离线记录见 logs/navigation-execution-20261020/。机器人恢复连接后，先完成独立 ROS 域中的真实控制器测试、核对停机状态并部署，再进行实车低速验证；当前不应通过反复点击目标绕过保护。

tools/check_navigation_controller.py 是后续隔离检查入口：固定 ROS_DOMAIN_ID=87、仅本机通信，使用模拟地图、雷达和里程计，只启动 Nav2 controller_server；不连接底盘 Worker、不使能电机。它不能替代真实舵轮动力学和带障碍物的实车验证。
