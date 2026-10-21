# PICO 接收与双臂遥操

模块位于 `scripts/pico4/`。全部命令从仓库根目录运行。

腕部联合限位默认开启；工作台 PICO 页可运行中切换，独立终端按 `g` 启用、`G` 停用。
切换先暂停并保持使能，双方确认后松开 Grip 再重握；当前 Home 会中止。
接近边界时缩小 TCP 目标、重新 IK 并丢弃多余手柄位移，机器人端独立复核。
详见 [腕部保护模块与适用范围](../../docs/腕部保护.md)。

夹爪遥操在工作台「PICO 遥操」页显式勾选「同时使能 TacCap 夹爪遥操」后生效。
命令行使用 `bash run.sh vr --grippers`，原 `bash run.sh vr` 仍只启用上半身。
Grip 保持双臂跟随；食指 Trigger 按压量线性映射为夹爪闭合程度，
`开度 = 该侧最大开度 × (1 - Trigger)`，最大开度默认 100%。
与手臂保持同一手柄对应关系：默认镜像配置中右手柄控制左臂/左爪。
左右 SN 留空按 SDK Left/Right 识别，也可分别指定固件 SN；缺少一侧不复用另一侧设备。

启动、Home、暂停和失去有效输入期间夹爪保持；启动与恢复时先松开 Trigger，再按压开始。
按压后松开对应设定的最大开度。启动和暂停恢复时先保持实测位置，即使当前位置高于设定上限，
也不会因启动自动闭合到上限。暂停取消当前夹爪行程但保持使能，退出请求夹爪下使能。
夹爪网络与串口控制在独立线程/进程运行，不阻塞双臂高频控制。
调试页和遥操共用 [夹爪接口](../../apps/gripper_control/README.md)。

夹爪速度现在可在 PICO 页调整，默认 0.75 rad/s（原为固定 0.25 rad/s），
命令行参数为 `--gripper-speed-rad-s 0.75`。这是电机轴速度，0.01–1.0 rad/s，
启动时生效；力矩仍为 1.1 Nm。夹爪目标发送和接收调度周期为 10 ms，
独立于双臂 450 Hz 循环；实际无线延迟需实测。

「左爪 / 右爪最大开度 %」分别设置 0–100%，启动时生效，运行中控件置灰。
例如上限为 60%：Trigger 全松、按一半、按到底分别对应 60%、30%、0%。
此设置只改变遥操目标映射，不改写夹爪固件标定，也不限制独立调试页的全行程。
反馈中显示相对闭合的电机转角：`上限 × (max_open_rad - min_open_rad)`，
它不是两根手指的夹角。命令行与界面共用同一映射：

```bash
bash run.sh vr --grippers --left-gripper-max-open-percent 60 --right-gripper-max-open-percent 60
```

工作台模型复用 PICO 会话的实测开度，显示消息目标为 10 Hz，不改变双臂控制频率。
超过 0.5 秒没有有效夹爪反馈时保留最后开度并变灰；指节形状是 CAD 的近似开合模型。

「双臂起始位姿」默认仍为原对称屈肘 Home；可选第二项「10-20 右臂实测镜像」，
或命令行传 `--home-preset right_mirrored`。`--home-file PATH` 仍可覆盖所选预设。
自动回 Home、右摇杆按压和界面 Home 按钮都使用同一个选择；运行中不能切换。
第二项来自 2026-10-20 14:18:38 读取的右臂 15 帧中值，左臂镜像符号为
`[-1,+1,-1,+1,-1,+1,-1]`，原始记录路径和角度保存在
[home_pose_right_mirrored.json](fairino/home_pose_right_mirrored.json)。
仅移动 J1–J14，沿用 8°/s 关节速度上限、至少 4 秒的 Home 轨迹及中止机制；
已校验正逆解、关节限位和镜像关系，未进行实机轨迹与碰撞验证。

「遥操 TCP」可选择法兰中心或夹爪固定指尖中心。先在「安装角度 / TCP
设置与预览」核对并保存左右实装角度、是否安装和固定工具点，再选择带夹爪模式。
命令行对应 `bash run.sh vr --tcp-tool taccap --grippers`。
未确认配置只用于模型预览，不能进入夹爪 TCP 实机遥操；原法兰模式不受影响。
CAD 依据、偏置和操作说明见 [夹爪工具模型](../../models/taccap/README.md)。

工作台还可显式勾选「启用身体竖直升降」，或使用
`bash run.sh vr --body-lift`（两者一起用：`bash run.sh vr --grippers --body-lift`）。
右摇杆上下控制竖直速度，回中保持当前高度；启动、暂停、Home 后需回中才能再次升降。
不需要握 Grip。右摇杆按压仍回双臂 Home；双臂灵敏度改为左手 X 双击降低、Y 双击提高。

工作台可勾选「同时启动底盘 · 回正并使能」，或独立运行：

```bash
bash run.sh vr --chassis --chassis-host 192.168.58.3 --chassis-linear 0.06 --chassis-angular 0.10
# 同时使用夹爪和已标定的身体升降
bash run.sh vr --chassis --grippers --body-lift
```

底盘默认不启动。勾选后先沿用底盘键控的轮组回正、使能流程，等待就绪、摇杆回中后才接受速度。
左摇杆上下/左右对应机器人自身前后/左右平移，不发送旋转指令；右摇杆左右对应左转/右转，
单独拨动时原地旋转。两杆同时拨动允许组合平移与旋转，命令行可加
`--chassis-rotation-priority` 改为旋转时停止平移。右摇杆上下仍只控制可选的身体升降。
平移方向不随头显朝向或左右手臂映射改变；左杆平移不包含航向闭环纠偏。

最大速度默认 0.06 m/s、0.10 rad/s，沿用键控上限 0.20 m/s、0.40 rad/s；摇杆幅度连续映射，
左杆使用径向死区，斜向速度不超过上限。双击调的是双臂位移/旋转灵敏度，不改变底盘限速。
底盘无需握 Grip，摇杆回中停车并保持使能。暂停、Home、腕部切换、手柄断流或上半身未就绪时归零，
恢复后需新的回中样本；超过 0.2 秒没有新摇杆样本也归零，重复旧帧不能续命。
退出遥操会同时请求底盘、上半身及本次夹爪会话下使能，并分别记录确认结果。

底盘适配器 [vr_chassis.py](fairino/vr_chassis.py) 通过
[axis_session.py](../../apps/fairino_keyboard/axis_session.py) 复用原键控连接和机器人 worker，
继续使用同一个厂家控制器、里程计及故障/通信保护；独立 `bash run.sh base` 仍可用，
但不能与遥操同时占用底盘。上位机后台线程发送底盘最新输入，不阻塞双臂 IK 循环。

升降只改变 J18–J20，复用身体程序的 `body_core.py` 和 `body_config.json`：
标定低位至高位约 425 mm，保持起始水平偏移与胸部倾角、头部、腰转和侧倾。
默认竖直速度 15 mm/s，同时按身体配置默认 5°/s 限制折叠关节指令变化率。
侧倾时下降到不允许侧倾的高度会停止，需先用身体面板回正侧倾。
双臂与升降合成同一二十轴目标，由同一个机器人端控制实例下发；
摇杆回中/输入断流后保持新高度，退出再下使能。双臂目标仍相对胸部，不做地面坐标补偿。

```bash
bash setup.sh all
bash run.sh pico check
bash run.sh pico install-apk
bash run.sh pico service
```

安装器在项目中准备 Python 环境、PC Service、APK、ADB 与 SDK 绑定；版本、URL、SHA-256 见 [assets.json](assets.json)。USB 用于首次安装，日常输入通过 Wi-Fi。

头显填写电脑地址，显示 WORKING，开启 Controller Tracking/Send，并关闭 A 键切换 Send。另一个终端：

```bash
bash run.sh pico monitor --duration 10
bash run.sh pico preview --sensitivity 0.25
bash run.sh errors
bash run.sh pico fairino --check
bash run.sh vr
```

monitor/preview 不连接机器人；preview 使用虚拟起点。开机有故障按 [故障排查](../../docs/故障排查.md) 单次清错，再复查归零。

`vr` 会使能并先回 Home，右手柄控制左臂、左手柄控制右臂。Grip 握住跟随、松开保持；A/X 按住冻结旋转，B/Y 按住冻结位移；左手 X 双击降低灵敏度、Y 双击提高（0.25 / 0.5 / 1）；左摇杆按压暂停；右摇杆按压 Home，再按中止。

双击要求两次均为不超过 0.25 秒的短按，两次之间不超过 0.35 秒，第二次松开才触发。
按住冻结立即生效，长按不会调灵敏度；短按期间也会暂时冻结对应分量。调档后暂停并保持，
双臂松开 Grip 后重握，底盘与身体摇杆回中后再拨动。左摇杆不再调整灵敏度，即使未启用底盘也一样。

| 组件 | 文件/目录 |
|---|---|
| 服务、APK、监视和预览 | [pico.py](pico.py) |
| 遥操主循环 | [vr_client.py](fairino/vr_client.py) |
| TCP 插值与 IK | [vr_cartesian.py](fairino/vr_cartesian.py)、[vr_plan.py](fairino/vr_plan.py) |
| 机器人执行 | [vr_robot.py](fairino/vr_robot.py) |
| Home | [vr_home.py](fairino/vr_home.py)、[home_presets.py](fairino/home_presets.py)、[原预设](fairino/home_pose.json)、[右臂镜像预设](fairino/home_pose_right_mirrored.json) |
| 清错 | [clear_errors.py](fairino/clear_errors.py) |
| 接收和 SDK 绑定 | [pico4/](../../src/lerobot/teleoperators/pico4/) |
| 双手柄 | [bi_pico4/](../../src/lerobot/teleoperators/bi_pico4/) |
| 运动学 | [TCP 模块](../../apps/fairino_tcp_probe/) |
| 上半身生命周期 | [上半身模块](../../apps/fairino_upper_probe/) |

`vr` 为 450/500/100 Hz、12 ms 缓冲；`pico fairino` 默认 90/200/100 Hz。先 TCP 位移线性/姿态 SLERP 插值再 IK，机器人没有额外 30 ms 关节追赶段。详见 [遥操与延迟](../../docs/PICO遥操有线方案与延迟说明.md)。

日志位于 `.local/pico4/fairino-logs/`。模型限位、逆解、急停、故障及断流检查仍有效。当前未提供完整 LeRobot 数据集录制接口。

松开 Grip 或升降摇杆回中后，程序发布保持目标，最多等待 3 秒确认：
收到发布之后的新硬件反馈、反馈的指令位置与保持目标一致（容差 `1e-5 rad`）、
全部关节速度不超过 `0.005 rad/s`，成功后继续保持使能。等待期间仍执行硬件反馈、
急停、故障和通信检查；不以固定 50 ms 后的旧反馈立即判失败。遥操状态回传 100 Hz
不代表每帧都包含新的硬件数据。

机器人日志中的 `stop_confirmation` 记录停止确认结果、等待时间、反馈年龄、
最大指令差及对应关节、最大速度及对应关节。本机额外保存 `*.exit.jsonl`，
在清理前记录退出来源及首个异常，清理后另记上半身和夹爪下使能确认。
夹爪的 `logs/gripper/*.jsonl` 分别记录心跳超时、ACK 异常、收到退出请求等原因，
避免将退出清理误报成通信超时。排查事件顺序优先使用单调时钟字段，
以免系统校时使墙钟时间回退。

```bash
bash run.sh test
bash run.sh pico fairino --link-check --observe-ros \
  --duration 25 --interpolation-hz 450 --receiver-hz 500
bash run.sh pico latency --duration 25
```
