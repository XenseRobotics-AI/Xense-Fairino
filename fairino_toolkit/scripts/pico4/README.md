> 2026-10-14 打包说明：本文件保留原工程的历史测试记录与命令。包内统一入口、当前默认配置和路径请以 [功能汇总](../../../文档/功能汇总.md) 为准；原工程 /home/lim 路径不是解压后的运行要求。当前版本为 TCP 插值 v3，旧 v2/30 ms 段落只描述历史版本。

# PICO 接收与相对遥操作准备

当前有线连接、450 Hz 实机结果及分段延迟见
[PICO 遥操有线方案与延迟说明](/home/lim/Documents/法奥/PICO遥操有线方案与延迟说明.md)（网络配置更新于 2026-10-11，测量于 2026-10-10）。

`monitor` 和 `preview` 用于头显数据接收及现有 `BiPico4` 的相对目标预览，
不会连接法奥或使能电机。预览的初始 TCP 是虚拟零点。
新增 `fairino --execute` 可驱动这台法奥的双臂，详见下方实机步骤。
这是独立的本地实机接入程序，尚未提供 LeRobot `Robot` 适配类或完整录制依赖。

## 故障检查与单次清错（2026-10-11）

开机或重启后，若启动遥操时出现全局 `0x0E10FF01`、关节 `J15=30` 等故障，先处理故障再启动遥操。历史记录中的 `30` 是各轴驱动器 EtherCAT 通信类故障码；故障轴以本次反馈为准。仓库根目录用 `bash run.sh errors` 检查，退出控制程序并确认静止、全部未使能、急停释放后，用 `bash run.sh errors --execute` 单次清错，随后再次检查错误码归零。若故障复发，保留日志排查。详见 [开机检查说明](../../../README.md)。

电脑运行以下命令；只需要系统 `python3` 和 `ssh`，本机不需要安装 ROS。
连接上半身 `192.168.58.1`（ROS Domain 0）和底盘 `192.168.58.3`（Domain 1），
SSH 用户为 `fairino`，密码由 SSH 在终端提示输入，不保存在代码或日志中。

```bash
cd /home/lim/lerobot-xense
# 默认只读检查上下身，不清错
python3 scripts/pico4/fairino/clear_errors.py

# 先退出遥操/键控，确认机器人静止、急停释放；依次清除上半身、底盘
python3 scripts/pico4/fairino/clear_errors.py --execute

# 也可以单独清除；底盘要求上半身故障转发已归零
python3 scripts/pico4/fairino/clear_errors.py --part upper --execute
python3 scripts/pico4/fairino/clear_errors.py --part base --execute
```

清错使用厂家 `/fairino_hardware_command_controller/command` Action 的
`reseterror()`，由控制器请求底层复位；不是改 ROS 故障话题，也不是直接写零到
`0x603F`。厂家文档将 `0x0E10FF01` 定义为“驱动器故障（全局广播）”，仍须看
各轴 `errorcode` 定位；当前的 `30` 是驱动器 EtherCAT 故障。
参考：[厂家状态看板与指令](https://fairino-art-doc.readthedocs.io/zh-cn/latest/PluginManual/plugin_board.html)。

脚本先检查关节反馈持续更新、未使能且静止、急停/上半身转发、控制器占用情况。
执行时与现有键控/遥操共用控制锁；每块板最多发一次复位。已无故障则跳过；
FATAL 故障、未释放的急停、非零或超时的上半身转发会阻止相应复位。
`--execute` 默认先上半身后底盘，上半身未成功则停止。
它不会使能、回 Home、发布运动目标、切换控制器、屏蔽急停或修改启动服务。
现场同时不要用网页或另一终端发送控制指令；文件锁只能约束使用同一锁的程序。

命令返回后继续观察 5 秒，要求最后至少 2 秒故障为零、驱动未使能且静止，才报告
成功。按 CiA 402 状态机判断使能，避免将 `0x220F/0x130F` 的故障反应状态误判
为已使能。`SUCCEEDED` 或 `0xC000` 本身不替代反馈验证。
清除后仍需另行启动控制程序；如果错误重现，保留日志排查原因，不循环清错。
连接超时或中断后，结果可能不确定，应先运行默认只读命令。

每次记录保存在 `scripts/pico4/fairino/diagnostics/*-fault-reset/`。
本次交付只进行离线测试和实机只读检查，没有代替用户运行 `--execute`。

离线回归检查（不会连接机器人）：

```bash
python3 -m unittest discover -s scripts/pico4/fairino -p 'test_fault_reset.py' -v
```

## 相对控制的含义

现有 `teleop_pico4.py` 的逻辑是：

1. 按住侧面的 Grip 时，记录手柄当前位置和当前目标 TCP。
2. TCP 目标 = 握持时目标 + 灵敏度 × 手柄相对位移（先换算方向）。
3. 姿态也在握持时对齐，以手柄后续的旋转变化更新目标。
4. 松开 Grip，目标保持；把手挪回舒适位置，重新握持不会把目标拉回去。

所以输入是相对变化，输出依然是控制器要执行的绝对 TCP 目标。
**仅做这种相对映射，不必知道头显到机器人的距离，也可暂不需要左右基座的平移量。**
但仍需确定手柄方向到各臂基座方向的旋转关系。若要输出同一上半身原点下的左右
TCP 绝对位置，则需要两臂完整的安装位姿。腕心到法兰/工具的偏移也仍需正确。

## 软件分别装在哪里

| 位置 | 软件 | 作用 |
|---|---|---|
| PICO 头显 | XRoboToolkit APK | 获取并发送手柄位置、姿态、Grip、Trigger 等 |
| 这台 Ubuntu 电脑 | XRoboToolkit PC Service | 发现/连接头显，向本机 SDK 提供数据 |
| 这台 Ubuntu 电脑 | `xensevr_pc_service_sdk` + BiPico4 | 接收数据并计算相对目标 |
| 这台 Ubuntu 电脑 | ADB | 通过 USB 安装头显 APK，常规 Wi-Fi 传输不需要一直插 USB |

仅控制双臂用头显和两只手柄即可，不需要额外的全身体感追踪器。
官方项目主要验证 PICO 4 Ultra；普通 PICO 4 还需核实系统与 APK 兼容性。
准备了标准 APK（最低 Android API 30）和 SDK29 版本（最低 API 29），安装入口
读取头显 Android API 后自动选择，不自动更新已有头显应用。以上最低版本已从
下载包中的 AndroidManifest.xml 核实，不仅根据文件名推断。

## 本机准备

适用于当前 Ubuntu 24.04 x86_64。下载和运行文件都保存在 `.local/pico4/`。
现有 Conda 用于创建独立 Python 3.12 环境，环境自身也位于这个目录；没有开机自启。
本地 C++ 接口使用项目现有 `libPXREARobotSDK.so`，依赖 CMake 和 C++ 编译器。

```bash
cd /home/lim/lerobot-xense
bash scripts/pico4/run.sh setup
bash scripts/pico4/run.sh check
```

`setup` 下载并校验官方发行包，解压 PC Service（不运行 deb 安装脚本），构建 Python
接口。GitHub 包的 SHA-256 固定在 `assets.json`；安装的 Python 版本列表在
`.local/pico4/requirements-installed.txt`。ADB 来自 Google 官方 Platform Tools。

## 在头显上安装

在头显系统中开启开发者模式及 USB 调试，用 USB 数据线连接电脑，并在头显内
允许这台电脑进行 USB 调试。然后执行：

```bash
cd /home/lim/lerobot-xense
bash scripts/pico4/run.sh install-apk
```

多台 Android 设备同时连接时用 `--serial 设备序列号` 指定 PICO。
如确定需要标准版，使用 `install-apk --variant standard`。
若应用已经存在，脚本会停止，不会自动卸载或覆盖其数据。

## 连接与检查

1. 电脑与头显连接同一局域网。当前有线方案在头显填写电脑有线口的局域网地址
   `192.168.31.152`；用 `ip -br -4 addr` 核对，DHCP 地址可能改变。`check` 显示的
   Wi-Fi 地址不一定是本次应选的地址。不要填机器人 `.1` / `.3`，也不要使用
   Docker 或代理虚拟网卡的地址。
2. 在第一个终端启动服务，保持运行：

   ```bash
   cd /home/lim/lerobot-xense
   bash scripts/pico4/run.sh service
   ```

3. 头显里打开 XRoboToolkit，选择/填写电脑的局域网 IP，连接成功应显示 `WORKING`。
   打开 **Controller Tracking** 和 **Send**。只连接但没打开 Send，不会有手柄数据。
   建议关闭用 A 键切换 Send 的选项，避免以后与遥操作的 A 键功能冲突。
4. 第二个终端查看原始数据：

   ```bash
   cd /home/lim/lerobot-xense
   bash scripts/pico4/run.sh monitor
   ```

5. 原始数据正常后，Ctrl+C 退出 monitor，再预览相对目标：

   ```bash
   bash scripts/pico4/run.sh preview --sensitivity 0.25
   ```

预览要求开始时先松开两个 Grip。输出为虚拟 TCP 位置（mm）和旋转的 6D 表示
`r1...r6`（旋转矩阵前两列，不是六个旋转自由度，也不是欧拉角）。
默认采用第一人称模式，可选择 `--mode mirror` 或 `--mode mirror_same_direction`。
预览不执行机械臂回零、腕部翻转、夹爪动作或录制命令。

`monitor`/`preview --duration 10` 可以进行有时限的检查；没有收到有效双手柄数据
会以退出码 2 结束。流时间戳超过 0.5 秒未变化时停止更新预览，恢复后要求先松开
Grip。现有 SDK 的时间戳属于整个消息流，不能用这个检查代替实机控制中的每只
手柄跟踪有效性检测、机器人状态反馈和急停保护。

## 离线验证

不需要服务、头显或机器人：

```bash
cd /home/lim/lerobot-xense
PYTHONPATH=src:scripts/pico4 XENSE_LOG_DIR="$PWD/.local/pico4/logs" \
  .local/pico4/env/bin/python -m unittest discover -s scripts/pico4 -p 'test_*.py'
```

## 法奥双臂实机遥操作

本机依赖现有 `/home/lim/Documents/法奥/fairino_tcp_probe` 和同级
`fairino_upper_probe`。可通过环境变量 `FAIRINO_TCP_DIR` 更改 TCP 程序目录。
复用此前验证过的厂家七轴 FK/IK、法兰中心 `[-106, 0, 0] mm` 偏移及 ROS
20 轴生命周期。当前为 TCP 插值 v3：不限制起点周围的活动范围。现在默认使能后先回指定双臂 Home（不是机械零位），
已取消上一版的应用层 TCP/关节限速、加速度上限和位置跳变阈值。
旧进程不会热更新，需要退出后重新启动；新版启动会显示“TCP 插值 v3”。

### TCP 插值前移到 IK 之前（2026-10-10）

实时链路为：PICO 原始帧 → 相对映射/灵敏度/冻结 → TCP 位移线性插值与姿态
四元数最短弧 SLERP → 每个采样点执行厂家 IK → SSH → 机器人单点位置保持。
电脑保留收到的原始帧，在每个下发周期按电脑接收时钟采样。每次 IK 优先以上一
次规划的关节解作为参考，减少反馈滞后引起的分支变化；不承诺跨奇异点时仍连续。

机器人端已删除原 30 ms 关节线性追赶段。ROS 消息只有一个 `time_from_start=0`
的位置点，消息头时间戳也为 0，表示收到即开始。JTC 保持这个采样点，直到下一个
采样点到来。已在机器人上用其实际安装的 JTC 动态库运行独立采样测试，确认首个
周期就返回目标、之后保持；测试没有创建 ROS 节点、发布话题或使能电机。
[ROS 对零时刻首点的定义](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)。

```bash
cd /home/lim/lerobot-xense
bash scripts/pico4/run.sh fairino --execute \
  --mode mirror_same_direction --rotation-mode mirror --hand-mapping crossed \
  --freeze-buttons --sensitivity 0.25 \
  --interpolation-hz 90 --interpolation-delay-ms 12
```

- `--interpolation-hz` 与 `--command-hz` 同义，同时控制 TCP 采样、IK、目标发送频率。
  默认 90 Hz；提高频率不会增加头显的真实采样信息，实际接收频率以日志为准。
- `--interpolation-delay-ms` 默认 12 ms，是明确的输入缓冲延迟。实时插值需要前后
  两个已收到的样本；没有未来样本时保持最近样本，不预测、不外推。设为 0 则取
  最新样本，此时通常没有可插入的中间点。它不是“每个新目标再用 12 ms 追过去”。
- Grip/按键不经过这段延迟。松握立即取消该侧目标，冻结切换时清除对应分量的
  待执行历史，松开冻结不会补走。暂停/重握清空旧缓冲。Home 继续使用已有的关节
  空间五次时间曲线与原回程时长；头部及身体保持逻辑不变。
- 旧 `--horizon-ms` 参数只接受 0，非零会明确报错。没有重新添加 TCP/活动臂速度
  上限或起点范围限制，模型关节范围、急停、断流和保持关节保护继续生效。

动作日志增加 `ik_ms`、`cartesian_sampling`、`pico_pc_sample_age_ms`。机器人状态增加
`command_trace`（读入、校验、发布、观察到相同硬件指令的时间）及
`hardware_observed_hz`。最后一项是机器人端实际观察到的硬件反馈频率，和 100 Hz
状态回传不同；观察到指令反馈也不代表电机已经物理到位。

### 2026-10-10 延迟实测与网络选择

当日稍后的 450 Hz 无运动 A/B 测试：无线实际处理约 230.2 Hz，有线约 438.3 Hz；
处理间隔最大值从 131.81 ms 降到 6.39 ms。刚才有线实机遥操稳定段下发约 449.5 Hz、
接收并发布 ROS 约 433.1 Hz，双臂本地整体计算中位数 1.24 ms（其中 IK 约 0.69 ms）。
详见 [有线方案和分段测量说明](/home/lim/Documents/法奥/PICO遥操有线方案与延迟说明.md)
及 [实机统计](fairino/diagnostics/20261010-174025-wired-teleop-summary.json)。
不同批次的网络、负载及测量边界不同；下面保留较早的 90 Hz 测试记录。

详见 [分段延迟与 Wi-Fi/有线对比](fairino/diagnostics/20261010-latency-report.md)。
25 秒、90 Hz、2250 个模拟双臂目标的无运动测试：

| 项目 | Wi-Fi | 有线 |
|---|---:|---:|
| ping 中位数 / P95（各 200 次） | 7.67 / 34.9 ms | 0.296 / 0.372 ms |
| ping 最大值 / 丢包 | 148 ms / 0 | 0.482 ms / 0 |
| 应用层确认往返中位数 / P95 | 17.71 / 34.66 ms | 6.93 / 12.47 ms |
| 有效处理目标（不订阅 ROS） | 2216 / 2250 | 2250 / 2250 |
| 有效处理目标（只读订阅 ROS） | 2057 / 2250 | 2248 / 2250，仅最初两帧被覆盖 |

电脑双臂 IK 中位数约 1.4–1.5 ms；机器人 FK 校验约 0.9–3.4 ms（不同测试负载下
变化）。主要长尾出现在无线/传输路径。另测 PICO 报文时间戳→电脑 SDK：中位数
9.4 ms、P95 15.2 ms、最大 41.5 ms，USB 校时误差约 ±5.4 ms，收到约 90 Hz。
这不含头显时间戳之前的追踪/预测，也不含真实电机响应；不能将各段中位数、往返
时延直接相加当作完整单程运动延迟。本次实测没有让机械臂运动。

2026-10-11 已将 `192.168.58.202/24` 永久保存到有线连接 `Wired connection 1`，
并从无线连接 `KBE_OUT` 删除 `192.168.58.201/24`。有线继续 DHCP 获取 `.31` 地址，
PICO 接收方式保留。主路由表的默认上网路由仍走 Wi-Fi，既有 FlClash 策略保持。
当前无需再添加临时地址，启动前可核对保存配置与实际路径：

```bash
nmcli -g ipv4.addresses connection show "Wired connection 1"
ip -br -4 addr
ip route get 192.168.58.1
# 预期 dev enp0s31f6 src 192.168.58.202；没有设置机器人网关。

# PICO 单程软件链路测量：需要 USB 调试，头显 Controller Tracking 和 Send 开启。
bash scripts/pico4/run.sh latency --duration 25
# 电脑—机器人通信测试，真实 FK/IK，模拟关节状态，不使能、不发布运动目标。
bash scripts/pico4/run.sh fairino --link-check --duration 25
# 额外只读订阅真实 ROS 硬件状态，仍不发布运动目标。
bash scripts/pico4/run.sh fairino --link-check --observe-ros --duration 25
```

配置恢复步骤与修改前后备份见上方有线说明；旧的临时地址添加 / 删除方法不再作为日常启动步骤。

先退出其它上半身控制程序，保持 PC Service 运行。头显连接电脑局域网 IP，
勾选 Controller 和 Send。启动前松开两侧 Grip：

```bash
cd /home/lim/lerobot-xense
bash scripts/pico4/run.sh fairino --execute --sensitivity 0.25
```

输入机器人 SSH 密码后，程序校验当前反馈和两臂正逆解，随后使能全部 J1–J20
并先回双臂 Home。默认第一人称下左手柄对应左臂 J1–J7，右手柄对应右臂 J8–J14；头部、腰部及
折叠结构 J15–J20 保持。每次握持以该臂当前反馈指令位置的法兰正解为起点。

| 操作 | 行为 |
|---|---|
| 按住某侧 Grip 并移动/旋转手柄 | 相对控制对应手臂；可双侧同时握持 |
| 松开某侧 Grip | 该臂停止追踪并保持当前位置，另一臂仍可控制；清除该侧未完成的目标 |
| 左手柄 X | 灵敏度循环 `0.25 → 0.5 → 1.0`；暂停两臂，松开 Grip 后重握生效 |
| B/Y 或终端空格 | 暂停双臂并保持；松开两侧 Grip 后重握继续 |
| 终端 `+` / `-` | 切换灵敏度，与 X 同样需要松握后重建起点 |
| Esc、Q、Ctrl+C | 结束本次控制，等待确认全部 20 轴下使能 |

### 面对面、镜像旋转、手柄冻结与 Home（真机入口，2026-10-09 更新）

以上按键表是默认模式。面对面同房间方向并启用独立冻结时，在调试工具包根目录运行：

```bash
bash scripts/pico4/run.sh fairino --execute --mode mirror_same_direction --rotation-mode mirror --hand-mapping crossed --freeze-buttons --sensitivity 0.25
```

原 `/home/lim/lerobot-xense` 目录也支持同一命令。该模式左手柄控制机器人右臂，
右手柄控制机器人左臂。头显追踪空间的前方需要朝向机器人，追踪空间上方对应
机器人的胸部上方；仍不是经过标定的地面坐标系。冻结的是当前法兰 TCP 目标，
不包括未建模夹爪的偏移。

| 参数 | 方向含义 |
|---|---|
| `--mode first_person`（默认） | 操作者前/左/上对应机器人胸部前/左/上 |
| `--mode mirror` | 面对面照镜子：操作者前/左/上对应机器人前/右/上；向机器人伸手时，机器人向操作者伸手 |
| `--mode mirror_same_direction` | 面对面同房间方向：操作者前/左/上对应机器人后/右/上 |
| `--rotation-mode mirror` | 旋转独立采用面对面镜像，围绕当前 TCP 点旋转，不改变位移映射 |
| `--rotation-mode auto`（默认） | 第一人称用同向旋转，两个面对面模式都用镜像旋转 |
| `--hand-mapping auto`（默认） | 第一人称同侧，两个面对面模式交换左右手臂 |
| `--hand-mapping same` / `crossed` | 显式指定同侧或交叉对应，不改变坐标方向 |

开启 `--freeze-buttons` 后，下表替代原 X 灵敏度与 B/Y 暂停功能：

| 操作（仍需握住该侧 Grip） | 该手柄所控制的手臂 |
|---|---|
| 按住右手 A / 左手 X | 冻结旋转，只跟随位移 |
| 按住右手 B / 左手 Y | 冻结位移，只跟随旋转 |
| 同一手柄两个键同时按住 | 位移和旋转都冻结 |
| 松开冻结按键 | 对应分量恢复跟随，不补走冻结期间的动作 |
| 松开 Grip | 对应手臂停止跟随；再次握持重新建立起点 |
| 左摇杆上 / 下 | 灵敏度增 / 减一档（0.25 / 0.5 / 1），回中后才能再次调节；松开双侧 Grip 后重握 |
| 右摇杆按压 | 回双臂 Home；回程中再次按压则中止 |
| 左摇杆按压 / 终端空格 | 暂停；正在回 Home 时中止回程 |
| 终端 `+` / `-` | 仍可调灵敏度，松开双侧 Grip 后重握 |
| 终端空格 | 暂停双臂，松开双侧 Grip 后重握 |
| Esc、Q、Ctrl+C | 退出并等待确认全部 20 轴下使能 |

这是**按住冻结、松开恢复**，不是单击切换。左右手柄独立生效；在交叉对应下，
X/Y 影响机器人右臂，A/B 影响机器人左臂。**头显 XRoboToolkit 中须关闭
“用 A 键切换 Send”的选项**，并保持 Controller Tracking 与 Send 开启。
终端每秒显示冻结分量；动作日志记录 `mode`、`controller_sides` 与每臂 `freeze`。
未传 `--freeze-buttons` 时保留旧按键行为。未增加速度/位置范围限制；模型限位、
急停、断流停止与既有使能流程保持。已做离线坐标、冻结连续性、按键路由和厂家
IK 回归；后续用户实机运行已有记录。最新 90 Hz 改动的验证仍限于离线和无运动通信检查。

### 双臂 Home 与启动行为

`fairino --execute` 每次使能后默认先回 Home，完成后才进入 Grip 遥操。Home
仅是约定的工作姿态，不是编码器清零或机械零位；J15–J20 始终保持启动姿态。
需要跳过本次启动回程时加 `--no-auto-home`，右摇杆按压回 Home 仍然可用。
配置文件为 [`fairino/home_pose.json`](fairino/home_pose.json)，也可用
`--home-file 路径` 指定另一份同模型的双臂角度配置。

| 手臂 | 七个关节角度（度） |
|---|---|
| 左臂 J1–J7 | 45, 75, −100, 85, 85, 0, 0 |
| 右臂 J8–J14 | −45, 75, 100, 85, −85, 0, 0 |

这是双臂在身体两侧对称屈肘、前臂向前的姿态，基于 2026-10-09 15:27 的实测
姿态选取。与该次读取相比，左/右臂最大单轴变化约 12.44°/9.80°。
末端在胸部坐标约为 `[408.1, ±403.5, 32.5] mm`，不是离地高度。
从该次读取到 Home 的路径每臂抽样 101 点，厂家 FK 与整机 URDF 一致，并在
模型关节范围内；记录在 [`fairino/diagnostics/20261009-home-validation.json`](fairino/diagnostics/20261009-home-validation.json)。
这些模型检查不包含环境障碍物和未建模夹爪。之后在 2026-10-09 15:54:35 的实机运行日志中记录了 Home 完成；该次使用升级 90 Hz 前的版本，新 90 Hz 运动尚未验证。

回 Home 使用两臂同步的五次时间曲线，至少 4 秒，规划关节峰值 8°/s；位移较大
时自动延长。这是自动回程的速度，不会重新加入遥操的速度或起点活动范围限制。
轨迹分段以同一 ROS 位置接口发送，保留 250 ms 请求新鲜度检查；不发送一个
断线后仍会完整执行的长轨迹。必须松开两侧 Grip 并等待双臂静止后才开始。
回程途中握任一 Grip、再按右摇杆、按左摇杆/空格、退出或断流都会中止；恢复
数据不会自行续走。到位需实际关节误差小于 0.2°、速度近零并稳定 0.3 秒。

### 2026-10-09 通信与更新频率检查

重新连接后 PICO 输入实测约 90 Hz，帧间隔中位数 11.0 ms，最大 34.9 ms，
10 秒内没有超过 250 ms 的断流。SDK 当前确实提供 `axisX/axisY/axisClick`；
旧真机接收器没有读取这些字段，本版用于灵敏度、暂停与 Home，不作为底盘命令。
电脑经 Wi-Fi `wlp0s20f3` 连机器人 `.1`；20 次 ping 无丢包，往返均值 4.91 ms、
最大 11.94 ms。更早一轮均值 14.42 ms、最大 86.15 ms，表明抖动会变化。
这不是从手柄动作到电机响应的完整端到端时延。

旧遥操日志的目标更新间隔中位数 67.5 ms，约 14.8 Hz，状态反馈约 18.8 Hz。
此前 20 Hz 版本的真机日志实测约 19.94 Hz，反馈约 18.72 Hz。
现已升级为下述 90 Hz 配置；500 Hz 是机器人底层控制循环配置，不是外部遥操频率的保证。

今天日志中 15:14:41 和 15:16:30 的两次中断确为“PICO 目标超过 0.25 秒未更新”；
这是应用层请求超时，不是 TCP 套接字报错。15:17:00 则是 J16 速度 0.594°/s
超过躯干保持阈值 0.5°/s，硬件错误码当时仍为零。后续 15:54:39 的最终退出同样是 J16 躯干速度检查触发：约 0.535°/s 超过 0.5°/s，
硬件错误为零，随后 15:54:40 确认全部下使能。其他暂停还包括 Grip 松开或输入暂停。Grip 是持续握持跟随，不是单击切换；超过 0.5 开始、降到 0.3 以下停止，
重新握持还需关节静止。新版动作日志补充 Grip、摇杆、暂停原因和旋转模式；没有
放宽超时或躯干保护。完整记录见 [`fairino/diagnostics/20261009-latency.json`](fairino/diagnostics/20261009-latency.json)。

### 历史记录：90 Hz 位置目标 v2（2026-10-09 16:30）

本节记录旧版；当前原工程以以上 v3 为准。旧工具包/压缩包未自动同步本次源码修改。

默认参数：目标下发 `90 Hz`，接收空闲轮询 `200 Hz`（新目标会提前唤醒），
状态回传 `100 Hz`，位置插值 `30 ms`。Home 也以 90 Hz 更新，但回程时长、
峰值速度与按键没有改变。镜像、交叉手柄、冻结和灵敏度均保留。
实际使用的是已有 ROS JointTrajectoryController；没有切换厂商 SDK 运动接口。

```bash
# 调试工具包根目录；运行中的旧进程必须先退出
bash run.sh pico fairino --execute \
  --mode mirror_same_direction --rotation-mode mirror --hand-mapping crossed \
  --freeze-buttons --sensitivity 0.25 --command-hz 90

# 不需要头显；模拟双臂目标走真实 SSH/JSON 链路，并在机器人上校验 FK
# 不初始化 ROS、不加载控制器、不使能、不发送真实关节目标
bash run.sh pico fairino --link-check --duration 15
```

原 `lerobot-xense` 目录把上述 `bash run.sh pico` 替换为 `bash scripts/pico4/run.sh`。
终端显示“下发 / 接收 / 回传 Hz”：接收是机器人端实际接受的新目标数，不把重复
心跳当作新动作。状态中的 `target_count/last_request` 与动作日志 `rates` 可用于核对。
发送采用绝对时刻调度；超期跳过已错过的时隙，只处理最新输入，不排队补走旧目标。

旧版可调参数：`--command-hz`、`--receiver-hz`、`--feedback-hz`、`--horizon-ms`。
旧版未显式指定插值时长时，取 30 ms 和两个下发周期中的较大值。只有运行旧备份
才能使用 `--command-hz 20 --receiver-hz 50 --feedback-hz 20 --horizon-ms 100`。
缩短插值会减轻追赶滞后，也会让目标变化的响应更直接；90 Hz 不是机械运动速度的倍率。

验证：65 项离线测试通过，包含双臂运动学、镜像冻结、Home、急停/断流，以及
有计算耗时的 90 Hz 调度和新目标提前唤醒。15 秒真实网络压测发送 1350 帧，
实测 89.997 Hz；机器人接受 1254 帧，83.574 Hz；状态回传约 100 Hz。
96 个旧目标被后到的最新目标覆盖，最大接收间隔 216 ms；这不等于丢失了 96 个
网络数据包。机器人端独立正解校验单帧最长约 3.70 ms。**当前 Wi-Fi 尚未达到
稳定的 90 Hz 有效接收，且该测试不包含 ROS 发布和电机响应；本次没有试动真臂。**
记录见 [`fairino/diagnostics/20261009-90hz-link-check.json`](fairino/diagnostics/20261009-90hz-link-check.json)。

10 月 9 日测试时电脑到机器人走 `wlp0s20f3` Wi-Fi，省电开启。关闭省电的对比未完成：sudo
需要本机密码，NetworkManager 不支持直接重应用该项；没有断开网络或修改持久配置。
可在电脑终端临时关闭后重新运行通信测试，不能预先保证它能消除抖动：

```bash
sudo iw dev wlp0s20f3 set power_save off
iw dev wlp0s20f3 get power_save
```

有线连接仍是排查无线抖动的另一种方式。250 ms 请求/反馈检查和既有躯干保持
保护保留；提高频率不会自动解决 J16 速度保护触发或网络长时间中断。

灵敏度同时作用于位移和旋转。`0.25` 表示手柄移动 4 cm 对应目标移动 1 cm；
旋转 20° 对应目标旋转约 5°。命令行可改为 `--sensitivity 0.5` 或 `1`。
已去掉上一版的平移 8 mm/s、旋转 2°/s、关节 2/4°/s、关节加速度 20°/s²
上限，以及附带的两臂实测速度 8°/s 中止阈值。不再支持 `--joint-speed`。
平移和旋转不再共同缩小目标；灵敏度缩放后的完整 TCP 目标交给厂家逆解。
也取消 2° 逆解跳变、100 mm 手柄单帧跳变和 1.1° 解算参考差阈值。

v2 曾使用 **30 ms 的关节位置插值**（90 Hz 配置）。10 月 10 日 v3 已改为 IK 前
TCP 插值与机器人单点保持，详见前面的当前配置。底层驱动自身的跟踪响应仍会
影响实际运动，不能把灵敏度当成硬性速度保障。
[ROS Humble JTC 的位置插值说明](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)。

默认手柄先转换为第一人称的 **X 向前、Y 向左、Z 向上**；面对面模式在此基础上
按上表转换方向。方向固定在头显追踪空间，
不随手柄外壳旋转。两臂的绝对 TCP 目标仍分别在各自基座坐标系内。

双臂现改为读取厂家 20260731 整机 URDF 的固定安装关系
（`art7-both-chest-urdf-20260731-v1`），替换先前左臂估算、右臂未补偿的版本。
胸部坐标原点沿用 `body_Link`，轴改为 X 前（原 −Y）、Y 左（原 +X）、Z 上。
左臂原基座 +X 向前下约 45°、+Y 向后下约 45°、+Z 向外/左；
右臂原基座 +X 向前下约 45°、+Y 向前上约 45°、+Z 向外/右。
下表为四舍五入后的方向关系，运行时使用 URDF 原始 rpy 精度；数值需再乘灵敏度：

| 手柄相对移动 | 左臂原基座中的 TCP 增量 | 右臂原基座中的 TCP 增量 |
|---|---|---|
| 向前 1 mm | X +0.7071、Y −0.7071 mm | X +0.7071、Y +0.7071 mm |
| 向左 1 mm | Z +1 mm | Z −1 mm |
| 向上 1 mm | X −0.7071、Y −0.7071 mm | X −0.7071、Y +0.7071 mm |

反方向取负。位移使用 `d_base = C @ d_teleop`；相对旋转使用
`R_base = C @ R_teleop @ C.T`，再左乘握持起点的法兰朝向，因此旋转目标
保持法兰中心位置。它不是单纯交换欧拉角，也不会旋转绝对 TCP 位置。
厂家文件中的 pitch 为 `0.7854 rad`，与理想 45° 只有约 0.000105° 的截断差；
先前估算左臂矩阵与新矩阵的总体旋转差约 0.000192°，主要变化是补齐右臂。
`mounts.py` 还提供完整胸部位姿转换：`tcp_in_chest` / `tcp_in_base`，包含安装
平移和单臂 URDF 基座到厂家 DH 原点约 0.945 mm 的偏置，避免重复加入 d1。
两个厂家 DH 原点在胸部坐标中约为 `[0,+62.5,329]` / `[0,-62.5,329] mm`。
相对遥操只需要安装旋转；安装平移用于统一的绝对 TCP 位姿，目标日志中的
`tcp_chest_mm` 是以 mm 为长度单位的 4×4 胸部法兰位姿矩阵。

这些是厂家 CAD 名义参数，并非实物标定精度。头显追踪的前方需与所需机器人
前方一致；两臂对齐的是胸部，胸部随腰部/折叠机构倾斜时不会自动保持地面对齐。
新整机文件的头腰编号、部分限位与实机记录不一致，**仅采用已核对的双臂安装链**；
没有把整机 URDF 的关节顺序、限位或动力学参数覆盖到控制器。型号与差异详见
[`models/README.md`](fairino/models/README.md)。
重启 `fairino --execute` 后生效，启动提示及目标日志会标明新版映射。
这里的修正只用于 `fairino` 实机入口，`preview` 仍是原 BiPico4 虚拟预览。

TCP 是法兰面中心，不包含实际夹爪长度；Trigger 暂不驱动夹爪。
当前没有完整上半身模型下的双臂互碰检查。

控制链路：PICO 数据 → 厂家 PC Service/C SDK 的完整控制器帧 → 相对法兰目标
→ TCP 线性/SLERP 插值 → 厂家 IK → SSH → 上半身 ROS `fairino_upper_identify` 单点保持 → 20 轴驱动。
两臂合并发送，J15–J20 始终使用保持目标；机器人端独立检查模型正解、模型
关节范围、硬件故障及外部急停。旧速度积分器的 0.004/0.006 rad 轨迹偏差
中止阈值不再用于该位置目标入口。躯干保持检查、启动位置校验及停止确认仍在。
2026-10-10 起，原工程 `fairino` 遥操入口的保持关节实测速度保护由 0.5°/s
调整为 1.0°/s：双臂运动时检查 J15–J20，暂停保持时也检查双臂；超过阈值
会结束控制。这是异常速度中止阈值，不是 TCP 速度或灵敏度。独立关节点动、
TCP 键控的保持速度默认值仍为 0.5°/s。重启遥操后生效，状态日志字段
`held_speed_limit_deg_s` 记录当前值。上文 2026-10-09 的 0.5°/s 为历史事件记录。
模型范围不是实测机械硬限位；厂家模型关节限位和控制器自身保护没有绕过。
输入缺失或超过 0.25 秒未更新会暂停，恢复后必须松开 Grip；持续丢失 3 秒退出。
未更新的机器人运动请求在 0.25 秒后停止；没有后续消息时，控制器保持最后一次
校验过的关节采样目标，不外推或积分出更远的目标。
接收器不沿用上一帧缺失的控制器数据；应用若提供跟踪有效标记也会检查。
若头显端不提供该标记，持续发送的冻结姿态无法仅凭时间戳判别为物理跟踪丢失。

检查和实机控制使用同一入口，默认只读：

```bash
bash scripts/pico4/run.sh fairino --check
# 临时加载、初始化并卸载 ROS 控制器；不使能、不发送运动轨迹：
bash scripts/pico4/run.sh fairino --prepare-check
```

状态及目标日志在 `.local/pico4/fairino-logs/`。启动时没有手柄数据或 Grip
未松开，会在连接机器人控制入口之前退出。其它控制程序占用时拒绝接管。
离线验证使用真实厂家 IK 库，但所有运动反馈和 ROS 调用均由模拟器提供：

```bash
PYTHONPATH=src:scripts/pico4/fairino XENSE_LOG_DIR="$PWD/.local/pico4/logs" \
.local/pico4/env/bin/python -m unittest discover -s scripts/pico4/fairino -p 'test_*.py'
```

独立整机 URDF 与厂家 ART7-R7 FK/IK 的离线对比（不启动设备连接）：

```bash
PYTHONPATH=src:scripts/pico4/fairino XENSE_LOG_DIR="$PWD/.local/pico4/logs" \
  .local/pico4/env/bin/python scripts/pico4/fairino/audit_mounts.py
```

报告写入 `.local/pico4/fairino-logs/validation-urdf-mounts.json`。

## 官方来源

- [PC Service v1.0.0（含 Ubuntu 24.04 包）](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)
- [PICO APK v1.1.1（含 SDK29 版）](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1)
- [头显连接和 Send 说明](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)
- [Android Platform Tools / ADB](https://developer.android.com/tools/releases/platform-tools)

完整 LeRobot 录制仍需增加法奥 `Robot` 适配类及数据集依赖；`preview` 本身不会
控制法奥，实机入口是上述 `fairino --execute`。
