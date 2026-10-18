# 上半身夹爪：编译、扫描、调用与捏合测试

本文记录 2026-10-18 在上半身实际验证的流程。SDK 已在上半身编译并安装，可以直接从“扫描设备”开始；修改 SDK 源码后才需要重新编译。

## 1. 连接与已验证环境

在电脑终端登录上半身：

```powershell
ssh fairino@192.168.58.1
```

以下 Bash、Python 命令均在 **SSH 登录后的上半身**执行。每次重新登录后设置这些变量：

```bash
cd /home/fairino/umi
PY=/home/fairino/umi/.venv/bin/python
PORT=/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5C96089765-if02
SN=TCGU01A28Z0015s
```

| 项目 | 本次实际值 |
| --- | --- |
| 上半身 | `192.168.58.1`，SSH 用户 `fairino` |
| 系统 | Ubuntu 22.04.5，ARM64 / `aarch64` |
| Python / GCC / CMake | 3.10.12 / 11.4.0 / 3.22.1 |
| OpenCV / pybind11 | 4.5.4 / 2.13.6 |
| SDK | TacCap-Gripper 0.4.3 |
| SDK 源码 | `/home/fairino/umi/vendor/TacCap-Gripper` |
| 源码 revision | `49d6a6ba587f13c97e145725034360a44d590013` |
| Python 环境 | `/home/fairino/umi/.venv` |
| 夹爪 | Follower，从爪；EL05 电机；SDK 扫描显示 Left |
| 夹爪固件 SN / 版本 | `TCGU01A28Z0015s` / 1.2.16 |
| USB 芯片 SN / VID:PID | `5C96089765` / `1a86:55d2` |
| 控制接口 | `...-if02`，本次对应 `/dev/ttyACM1` |

USB 芯片 SN 与固件 SN 是两个编号。优先使用 `/dev/serial/by-id/` 路径，重新插拔后 `ttyACM` 编号可能变化。换夹爪后重新扫描，不直接沿用上表 SN。

本次 Windows 电脑保留 DHCP 上网，并增加 `192.168.58.200/24` 地址连接机器人；DHCP/static coexistence 在 active、persistent 两个配置中均已启用。现有连接正常时无需重复配置。通用网络说明见 [网络配置与连接](../docs/机器人网络配置与连接说明.md)。

## 2. 怎么编译

### 现有上半身重新编译

依赖缓存已在 `/home/fairino/umi/wheels/`。从 `umi` 目录执行：

```bash
cd /home/fairino/umi
SDK_BUILD_JOBS=2 bash build_sdk.sh
bash verify_sdk.sh
.venv/bin/python -m unittest -q test_xtac_wire.py test_xtac_grip.py
```

[build_sdk.sh](build_sdk.sh) 会校验 `wheels/SHA256SUMS`、安装构建依赖、编译 C++ 库和 Python 绑定、生成 wheel、安装到 `.venv`，最后验证导入。它使用 `nice -n 10` 和默认 2 个编译任务；编译前结束夹爪控制程序。

脚本内部实际构建命令如下，通常直接运行脚本即可：

```bash
cd /home/fairino/umi/vendor/TacCap-Gripper
nice -n 10 env CMAKE_GENERATOR='Unix Makefiles' CMAKE_BUILD_PARALLEL_LEVEL=2 \
  /home/fairino/umi/.venv/bin/python -m pip wheel . \
  --no-deps --no-build-isolation --no-index \
  --wheel-dir /home/fairino/umi/dist \
  --config-settings=cmake.define.TACCAP_BUILD_TESTS=ON \
  --config-settings=cmake.define.TACCAP_BUILD_EXAMPLES=ON \
  --config-settings=cmake.define.pybind11_DIR="$(/home/fairino/umi/.venv/bin/python -m pybind11 --cmakedir)" \
  --verbose
```

这个单独的 `pip wheel` 命令只生成 wheel；`build_sdk.sh` 还会执行安装。

### 新机器准备依赖

本次使用 Ubuntu 系统开发库和独立 venv；上游 README 推荐 Python 3.12，但这份源码的 `pyproject.toml` 支持 Python ≥3.10，本次已验证 3.10。不要把 Windows 的 `.venv` 复制到上半身。

新装 Ubuntu 22.04 ARM64 时，先准备系统依赖；现有上半身已经具备这些依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev python3-venv \
  libopencv-dev libspdlog-dev libfmt-dev libgtest-dev \
  python3-numpy python3-pytest usbutils
cd /home/fairino/umi
python3 -m venv --system-site-packages .venv
```

上半身当时无法访问 PyPI，所以构建使用离线 wheel。已有缓存可从 Windows 项目目录 `.local/upper-sdk-transfer/wheels/` 复制；也可以在联网的 Linux / Python 3.10+ 机器上准备以下纯 Python/头文件 wheel：

```bash
mkdir -p wheels
python3 -m pip download --only-binary=:all: --no-deps --dest wheels \
  pip==25.3 packaging==24.2 scikit-build-core==0.11.6 \
  pybind11==2.13.6 pyserial==3.5 pathspec==0.12.1 \
  tomli==2.2.1 exceptiongroup==1.2.2
(cd wheels && sha256sum *.whl > SHA256SUMS)
scp -r wheels fairino@192.168.58.1:/home/fairino/umi/
```

把本目录的源码、`build_sdk.sh` 和 `verify_sdk.sh` 一起放到上半身 `umi/`，然后执行前面的编译命令。系统 OpenCV、GTest 等依赖不包含在这个 wheel 缓存中。

这里固定 `scikit-build-core==0.11.6`：此前 0.10.7 不能解析这份项目的 SPDX license 字符串；系统旧版 pip 也不支持本次所用的 `--config-settings`。

### 产物与验证

| 内容 | 上半身路径 |
| --- | --- |
| 安装包 | `dist/taccap_gripper-0.4.3-cp310-cp310-linux_aarch64.whl` |
| CMake 构建目录 | `vendor/TacCap-Gripper/build/cp310-cp310-linux_aarch64/` |
| C++ 共享库 | 上述目录内的 `cpp/libtaccap_core.so*`，也随 wheel 安装 |
| 编译日志 | `build-logs/build.log` |
| C++ / Python 测试日志 | `build-logs/cpp-tests.log`、`build-logs/sdk-python-tests.log` |

```bash
cd /home/fairino/umi
.venv/bin/python -c 'import xense.taccap as t; print(t.__version__); print(t.__file__)'
sha256sum dist/taccap_gripper-0.4.3-cp310-cp310-linux_aarch64.whl
```

本次 wheel 的 SHA256 为 `456405bf892bd0bb40c5fd8a114733a3ce6ed50da68ecea66fc3e5030ccf2cdb`。这是本次产物的校验值，重新编译后的字节内容不一定相同。该 wheel 供 Linux ARM64 / CPython 3.10 使用。

[verify_sdk.sh](verify_sdk.sh) 验证已安装的 SDK，并运行 C++、Python 测试。上游 Python 测试要求源码目录能找到扩展，脚本会将已安装的 `.so` 链接过去；它不会覆盖已有的不同二进制。此脚本固定当前 `cp310/linux_aarch64` 构建目录与 SDK 0.4.3，换平台或版本需相应调整。

本次通过：**471 项 C++、134 项 Python、17 项本地串口封装测试**。另有 2 项真实 IMU 测试被排除，这些数字不代表 IMU、相机或触觉采集已完成验收。

## 3. 怎么扫描设备

扫描、诊断与运动程序依次运行，避免同时占用同一个夹爪串口。

### USB 和串口层

```bash
lsusb
ls -l /dev/serial/by-id/
cd /home/fairino/umi
.venv/bin/python xtac_probe.py --list
```

`--list` 只枚举 `1a86:55d2` 串口，不打开设备。本次看到两个串口 `ttyACM0`、`ttyACM1`，控制接口是 `if02`。只看到 USB 集线器、没有 `1a86:55d2` 时，检查夹爪 24V 供电及夹爪端 USB 数据线。

### 官方 SDK 扫描

```bash
cd /home/fairino/umi
.venv/bin/python - <<'PY'
from xense.taccap import scan_grippers
for d in scan_grippers():
    print(d.side.name, d.role.name, d.firmware_sn, d.mcu_device)
PY
```

本次实际输出：

```text
Left Follower TCGU01A28Z0015s /dev/serial/by-id/usb-1a86_USB_Dual_Serial_5C96089765-if02
```

SDK 扫描会打开可用控制串口查询身份，不使能电机。接多只夹爪时按固件 SN 明确选择目标。

### 读取详细诊断

```bash
cd /home/fairino/umi
PORT=/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5C96089765-if02
.venv/bin/python xtac_probe.py --port "$PORT" \
  --output "output/probe-$(date +%Y%m%d-%H%M%S).json"
```

[xtac_probe.py](xtac_probe.py) 只查询版本、身份、电机型号、标定、包络、状态与故障；不下发运动、清错、标定或固件写入命令。输出同时保留原始协议数据。

### 权限与串口占用

本次已经将 `fairino` 加入 `dialout`。新机器遇到 `Permission denied` 时：

```bash
sudo usermod -aG dialout fairino
# 退出 SSH，重新登录后检查
id
```

本次也已配置以下规则，避免 ModemManager 探测夹爪串口；新机器需要时再添加：

```bash
sudo tee /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules >/dev/null <<'EOF'
# TacCap control MCU: do not probe its serial ports as modems.
ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d2", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules
# 停止控制程序后重新插拔 USB，再检查实际控制串口
udevadm info -q property -n /dev/ttyACM1
sudo fuser /dev/ttyACM0 /dev/ttyACM1
```

属性中应有 `ID_MM_DEVICE_IGNORE=1`。`fuser` 若显示进程，先确认是什么程序并正常退出。本次没有禁用系统 ModemManager 服务。

## 4. 怎么调用 SDK

### Python 读取状态，不使能电机

以下代码已在上半身运行验证：

```bash
cd /home/fairino/umi
.venv/bin/python - <<'PY'
from xense.taccap import scan_grippers, FollowerGripper

sn = "TCGU01A28Z0015s"
matches = [d for d in scan_grippers() if d.firmware_sn == sn]
if len(matches) != 1:
    raise RuntimeError(f"期望找到一只 {sn}，实际找到 {len(matches)} 只")

with FollowerGripper(mcu_device=matches[0].mcu_device) as g:
    s = g.motor.read_status()
    f = g.motor.fault_report()
    c = g.motor.control_stats()
    print("固件:", g.firmware_version)
    print("归一化开口:", g.position())  # 0=闭合，1=张开
    print("位置 rad:", s.actual_pos, "速度 rad/s:", s.actual_vel)
    print("反馈力矩 Nm:", s.actual_torque, "温度 C:", s.motor_temp_c)
    print("状态位:", s.status)
    print("电机当前/历史故障:", f.motor_fault_code, f.motor_latched_fault_code)
    print("固件当前/历史故障:", f.firmware_fault_code, hex(f.firmware_latched_fault_code))
    print("控制周期超时累计:", c.deadline_miss_count)
PY
```

`with` 结束会释放串口；`g.close()` 本身不负责电机失能。运动代码必须停止控制器、卸力和失能后再关闭连接。控制器运行时通过 `controller.snapshot()` 取反馈，不同时调用 `motor.read_status()` 或启动另一个串口诊断程序。

SDK 的流式读取示例是 [follower_status.py](vendor/TacCap-Gripper/python/examples/follower_status.py)：

```bash
cd /home/fairino/umi
.venv/bin/python vendor/TacCap-Gripper/python/examples/follower_status.py \
  TCGU01A28Z0015s --seconds 2 --hz 100
```

该示例查询版本、启动状态流后停止，不使能夹爪；版本查询在部分电机协议下可能触发停止，因此同样只在空闲时运行。本次另行测得状态流约 99.56 Hz，MCU 状态时间戳随之推进。失能且未开流时，单次诊断中的时间戳可能不推进，应结合流式结果判断。

### 官方 SDK 位置控制示例：会实际运动

此前 SDK 验收执行的是以下参数，先闭合，再张开到 80%：

```bash
cd /home/fairino/umi
.venv/bin/python vendor/TacCap-Gripper/python/examples/force_position_control.py \
  TCGU01A28Z0015s --grasp-torque 0.35 --close-speed 0.2 --targets 0.0,0.8
```

[force_position_control.py](vendor/TacCap-Gripper/python/examples/force_position_control.py) 使用 `ForcePositionConfig.for_spec()`、`ForcePositionController`、`set_target()` 和 `snapshot()`，结束执行 `stop()` / `disable()`。该官方示例启动时会调用 `motor.clear_fault()`，运行前先保存诊断记录；本次后续的 1.1 N·m 捏合使用下一节的独立脚本。

## 5. 我们的 1.1 N·m 捏合怎么执行

### 常用命令

开合区域保持空。确认无其他程序控制这只夹爪后，先做不运动的预检：

```bash
cd /home/fairino/umi
PORT=/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5C96089765-if02
.venv/bin/python xtac_grip.py --port "$PORT"
```

输出 `Preflight OK. No motion.` 表示脚本检查通过。实际执行一次：

```bash
.venv/bin/python xtac_grip.py --port "$PORT" --execute --hold 2
```

**动作是闭合 → 稳定夹持 2 秒 → 下发零力矩 → 失能。没有主动张开的步骤。** `Ctrl+C` 会进入清理流程；若输出 `DISABLE NOT CONFIRMED`，按脚本提示切断夹爪 24V 并检查链路。

### 脚本参数与执行逻辑

[xtac_grip.py](xtac_grip.py) 使用 `pyserial` 和项目协议封装直接通信，依赖同目录的 `xtac_probe.py`、`xtac_upgrade.py`（仅使用串口类）、`xtac_motion_test.py`、`xtac_full_close.py`。运行它不要求导入已编译的原生 SDK。

| 项目 | 当前实现 |
| --- | --- |
| 力矩指令 | `TORQUE = 1.1` N·m，闭合行程和接触阶段均使用该前馈设定 |
| 速度阻尼 | `DAMPING = 4.0`，运动时阻尼会改变净力矩请求 |
| 保持时长 | `--hold 2`，从稳定接触开始计时；检测到滑动会重新计时 |
| 身份限制 | 固定 SN `TCGU01A28Z0015s`、EL05、固件 1.2.16 |
| 前置条件 | 电机失能、标定完成、包络有效、当前故障清零、控制任务空闲 |
| 历史记录处理 | 接受并记录已存在的 `0x0303`；其他固件历史和电机历史故障会阻止启动 |
| 反馈 | 100 Hz；检查数据有效性、温度、状态位、标定行程和反馈间隔 |
| 退出 | 稳定保持结束、超时、反馈异常或中断时，尝试零力矩和失能 |
| 日志 | 自动保存 `output/grip-年月日-时分秒-微秒.json` |

当前脚本关闭了主机侧“反馈力矩/速度幅值”停机阈值，保留固件包络和上述检查；固件存储的连续/峰值包络为 1.1 / 1.8 N·m。**1.1 是指令设定，不保证反馈恰好为 1.1，也不等同于夹持力 N。** 两次保持阶段实际反馈约 1.20 N·m。

脚本不会清除故障历史或写入固件、标定、包络。更换设备时需要重新确认身份、型号和参数；当前脚本不是自动适配任意夹爪的入口。

### 本次通过 SSH 实际使用的包装方式

下面在同一脚本外加了 40 秒上限、SIGHUP/SIGTERM 转为清理中断，以及终端日志。它仍只执行一次 2 秒保持：

```bash
cd /home/fairino/umi
set -o pipefail
mkdir -p build-logs
run_id=$(date +%Y%m%d-%H%M%S)
timeout --signal=INT --kill-after=5s 40s .venv/bin/python -u - <<'PY' 2>&1 | tee "build-logs/grip-1p1-$run_id.log"
import runpy
import signal
import sys

def interrupted(signum, frame):
    raise KeyboardInterrupt("Signal " + str(signum))

signal.signal(signal.SIGHUP, interrupted)
signal.signal(signal.SIGTERM, interrupted)
sys.argv = ["xtac_grip.py", "--port",
            "/dev/serial/by-id/usb-1a86_USB_Dual_Serial_5C96089765-if02",
            "--execute", "--hold", "2"]
runpy.run_path("xtac_grip.py", run_name="__main__")
PY
```

超时和信号处理用于尝试正常清理，不能保证断电、进程被强制杀死或链路故障时收到失能确认。执行结束后检查 JSON 中的 `cleanup.disable_ack`、`final_status.status_bits` 和当前故障；必要时再用 `xtac_probe.py` 读取卸力后的静止状态。

## 6. 怎么看结果和故障

在上半身查看最新一次捏合报告，以下命令不连接设备：

```bash
cd /home/fairino/umi
.venv/bin/python - <<'PY'
import json
from pathlib import Path

paths = sorted(Path("output").glob("grip-*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
for p in paths:
    r = json.loads(p.read_text())
    if r.get("executed"):
        break
else:
    raise SystemExit("没有已执行的捏合报告")
print("日志:", p)
for key in ("grip_completed", "grip_result", "cleanup", "final_status", "final_fault", "error"):
    print(key, r.get(key))
before = r.get("control_before", {}).get("deadline_miss_count")
after = r.get("control_after", {}).get("deadline_miss_count")
print("控制周期超时累计:", before, "->", after)
PY
```

`grip_completed=true` 说明夹持步骤完成；`cleanup.disable_ack=true` 且 `final_status.status_bits=0` 表示本次收到失能确认并读到失能状态。还要单独检查故障和控制统计，不能只看退出码或“完成”文字。

本次记录如下，文件名来自上半身时钟：

| 测试 | 结果 | 控制周期超时累计 |
| --- | --- | --- |
| SDK 0.35 N·m，闭合→80% 张开 | 目标到位，结束失能，29°C | 0 → 1 |
| `xtac_grip.py` 1.1 N·m，保持 2 秒 | 完成，反馈均值约 1.200 N·m，结束失能，29°C | 1 → 2 |
| 同参数再执行一次 | 完成，反馈均值约 1.202 N·m，结束失能，31°C | 2 → 3 |

三次动作后当前电机/固件故障均为 0，但固件锁存 `0x0303`。源码枚举 [FirmwareFaultCode](vendor/TacCap-Gripper/cpp/include/taccap/protocol/payloads.hpp) 将它定义为 `ControlDeadline`，对应 MCU 控制周期超时；`deadline_miss_count` 是累计计数。原因尚未确认，不能据此声称完全无故障通过，也不能认定是 SSH 或上半身调度导致。历史码未清除；每次比较计数才能发现相同历史码下又出现的新超时。

主要实测记录都在上半身 `/home/fairino/umi/` 下：

- `build-logs/pinch-test-20261018.log`：SDK 0.35 N·m 测试。
- `output/grip-20261018-155721-543569.json`、`build-logs/grip-1p1-20261018-155721.log`：首次 1.1 N·m。
- `output/grip-20261018-155842-376043.json`、`build-logs/grip-1p1-20261018-155842.log`：再次 1.1 N·m。
- `output/post-grip-1p1-20261018-155842.json`：最后一次卸力后的独立只读复查。

Windows 本地备份在项目 `.local/upper-sdk-transfer/artifacts/`；该目录被 Git 忽略。全流程状态记录在 `.local/upper-sdk-transfer/task-state.json`，其中没有 SSH 密码。

## 7. 常见问题

| 现象 | 排查方向 |
| --- | --- |
| `ssh` 连接不上 | 先检查上半身、桥接设备供电及电脑到 `192.168.58.1` 的路由；网络正常不等于 USB 夹爪已接入 |
| 只有集线器，没有夹爪 MCU | 检查 24V 和夹爪端 USB 数据线，重新运行 `lsusb`、`--list` |
| SDK 扫描为 Unknown / 固件 SN 为空 | 检查 `dialout` 权限、重新登录、串口占用及 USB 连接 |
| `Device or resource busy` | 检查其他 SDK/控制进程及 ModemManager 规则 |
| 导入 `xense.taccap` 失败 | 使用 `/home/fairino/umi/.venv/bin/python`，确认安装的是 ARM64 / CPython 3.10 wheel，并查看动态库依赖 |
| 断电后增量编译出现 undefined reference | 先查看构建日志及目标文件完整性；本次曾发现 4 个零字节 `.o`，仅移除已确认损坏的生成文件后重编译成功 |
| 退出码为 0，但仍有 `0x0303` | 本脚本允许已有的这个历史码；查看前后 `deadline_miss_count`，保留 JSON 分析 |

更多 SDK API、C++ 用法见 [上游说明](vendor/TacCap-Gripper/README_CN.md)、[安装说明](vendor/TacCap-Gripper/docs/INSTALL.md)；项目实际 1.1 N·m 动作以 [xtac_grip.py](xtac_grip.py) 为准。
