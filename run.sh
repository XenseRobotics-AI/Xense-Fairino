#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export FAIRINO_PYTHON="${FAIRINO_PYTHON:-$ROOT/.venv/bin/python}"
if [[ ! -x "$FAIRINO_PYTHON" ]]; then export FAIRINO_PYTHON=/usr/bin/python3; fi
export FAIRINO_TCP_DIR="${FAIRINO_TCP_DIR:-$ROOT/apps/fairino_tcp_probe}"
COMMAND=${1:-help}
if (($#)); then shift; fi
case "$COMMAND" in
  console) exec "$FAIRINO_PYTHON" "$ROOT/apps/workbench/app.py" "$@" ;;
  teleop) exec "$FAIRINO_PYTHON" "$ROOT/apps/workbench/app.py" --page teleop "$@" ;;
  gripper) exec "$FAIRINO_PYTHON" "$ROOT/apps/gripper_control/app.py" "$@" ;;
  status) exec "$FAIRINO_PYTHON" "$ROOT/tools/inspect_robot.py" "$@" ;;
  network) exec "$FAIRINO_PYTHON" "$ROOT/tools/network_check.py" "$@" ;;
  base) exec bash "$ROOT/apps/fairino_keyboard/start.sh" "$@" ;;
  joints) exec bash "$ROOT/apps/fairino_upper_probe/start.sh" "$@" ;;
  body) exec bash "$ROOT/apps/fairino_upper_probe/start_body.sh" "$@" ;;
  tcp) exec bash "$ROOT/apps/fairino_tcp_probe/start.sh" "$@" ;;
  lidar) exec bash "$ROOT/apps/lidar_viewer/start.sh" "$@" ;;
  map) exec bash "$ROOT/apps/map_viewer/start.sh" "$@" ;;
  navigation) exec "$FAIRINO_PYTHON" "$ROOT/tools/navigation_mode.py" "$@" ;;
  mapping) exec "$FAIRINO_PYTHON" "$ROOT/tools/map_mode.py" "$@" ;;
  pico) exec bash "$ROOT/scripts/pico4/run.sh" "$@" ;;
  vr)
    exec bash "$ROOT/scripts/pico4/run.sh" fairino --execute \
      --mode mirror_same_direction --rotation-mode auto --hand-mapping auto \
      --freeze-buttons --sensitivity 0.25 --interpolation-hz 450 --receiver-hz 500 \
      --interpolation-delay-ms 12 "$@" ;;
  errors) exec "$FAIRINO_PYTHON" "$ROOT/scripts/pico4/fairino/clear_errors.py" "$@" ;;
  reset-upper) exec "$FAIRINO_PYTHON" "$ROOT/tools/reset_upper_once.py" "$@" ;;
  diagnostics) exec bash "$ROOT/scripts/run_diagnostics.sh" "$@" ;;
  test) exec bash "$ROOT/tests/run_tests.sh" "$@" ;;
  help|-h|--help)
    cat <<'HELP'
法奥机器人控制项目
  bash run.sh console                   集成工作台：连接、模型、键控、地图、遥操
  bash run.sh console --demo            工作台离线预览，不连接机器人
  bash run.sh teleop                    打开遥操界面，不自动使能
  bash run.sh vr --grippers --body-lift  PICO 显式启用 Trigger 夹爪和右摇杆升降
  bash run.sh vr --tcp-tool taccap --grippers --gripper-speed-rad-s 0.75  已核对安装后的指尖 TCP 遥操
  bash run.sh gripper                   独立夹爪调试：扫描、开度、闭合、保持
  bash setup.sh all                     安装所有功能的项目内环境
  bash setup.sh base                    安装键控/诊断 Python 环境
  bash setup.sh pico                    安装 PICO 接收/遥操 Python 环境
  bash run.sh network                   查看 IP、路由和机器人端口
  bash run.sh status                    只读状态、急停、故障、雷达
  bash run.sh errors                    只读故障检查
  bash run.sh errors --execute          单次清错：上半身→底盘，不使能
  bash run.sh errors --part base --execute  仅底盘清错
  bash run.sh base --check              底盘只读检查
  bash run.sh base                      底盘回正与 WASD/JL 键控
  bash run.sh joints --group left       左臂 J1–J7 单轴控制
  bash run.sh joints --group right      右臂 J8–J14 单轴控制
  bash run.sh joints --group body       头部/身体 J15–J20 单轴控制
  bash run.sh body --demo               身体高度/姿态离线界面
  bash run.sh body --check              身体程序只读检查
  bash run.sh body                      身体高度/姿态联动
  bash run.sh tcp --arm left            胸部坐标 TCP 键控，窗口可切臂
  bash run.sh lidar                     双雷达实时扫描图
  bash run.sh map                       地图窗口：建图、地图库、已有地图定位
  bash run.sh navigation status         二维导航状态；start 启动规划，drive 显式使能执行
  bash run.sh navigation plan --x X --y Y --yaw R   只规划，不移动
  bash run.sh navigation goal --x X --y Y --yaw R   发送导航目标（米/弧度）
  bash run.sh navigation cancel         取消导航目标
  bash run.sh mapping status            查看建图/静态地图/定位状态
  bash run.sh mapping maps              列出已保存地图（不自动选用）
  bash run.sh mapping localize --wait 90 启动已有地图定位，随后设置初始位姿
  bash run.sh mapping stop-localization 停止定位，保留底图
  bash run.sh mapping start --wait 90    显式准备新建图，等待有效里程计输入
  bash run.sh mapping pause             暂停建图，保留工作地图
  bash run.sh mapping save --name NAME   地图另存到新目录
  bash run.sh mapping view              归档工作地图并返回已有静态地图
  bash run.sh pico check                PICO 环境检查
  bash run.sh pico install-apk          USB 安装头显应用
  bash run.sh pico service              启动接收服务，保持终端运行
  bash run.sh pico monitor              查看手柄输入
  bash run.sh pico preview              虚拟相对目标预览
  bash run.sh pico fairino --check       机器人/运动学只读检查
  bash run.sh pico latency --duration 25  PICO 时间戳至电脑的链路测量
  bash run.sh vr                        450 Hz 面对面遥操，自动使能并先回 Home
  bash run.sh vr --mode first_person    第一人称遥操，同侧手柄控制同侧手臂与夹爪
  bash run.sh vr --chassis              同时回正并使能底盘，左杆平移、右杆左右旋转
  bash run.sh diagnostics geometry      雷达机械位置离线分析
  bash run.sh diagnostics lidar         双雷达样例配准与验证
  bash run.sh diagnostics wrist         手腕模型离线绘图
  bash run.sh test                      离线回归测试，不连接机器人

普通 base / joints / body / tcp 启动会使能；vr 会使能并自动回 Home。
vr 默认面对面：右手柄→左臂，左手柄→右臂；first_person 为同侧。Grip 握住跟随，松开保持。
A/X 按住冻结旋转；B/Y 按住冻结位移；X 双击降灵敏度、Y 双击升灵敏度。
左摇杆按压暂停；右摇杆按压 Home；--chassis 启用左杆全向平移、右杆左右旋转。
旧上半身清错兼容入口：bash run.sh reset-upper --execute-reset。
完整说明：docs/功能汇总.md；全部命令从仓库根目录运行。
HELP
    ;;
  *) echo "未知命令：$COMMAND；运行 bash run.sh help" >&2; exit 2 ;;
esac
