# 达显灵巧手集成方案（PICO 遥操末端选择）

分支：`feat/dx-hand`（基于 `ec52e81`）。状态：2026-11-19 用户已敲定，开始执行阶段 1–2。阶段 3 等 `feat/pink-ik` 完成并 merge 进本分支后再做。

## 目标

PICO 遥操界面可选末端：**法兰 / TacCap 夹爪 / 达显灵巧手**。选灵巧手时，手臂仍由 PICO 驱动（IK 可选，后续用 `feat/pink-ik` 的 `fairino_qp`），手指由 Manus 手套经达显 retargeting 驱动。灵巧手 USB 直连上位机。

与 TacCap 的区别：TacCap SDK 在法奥控制器上，通过 SSH 会话控制；达显手的 SDK 与 retargeting 在**上位机本地**运行，不经过机器人端代码。

## 已确认的决定

1. **上游使用 `05_DXHand/TsenseIV2C-main`（`cf42047`）**。其右手真机日志使用 `TsenseI_v3_2`、输入 `manus`（`manus_sdk`）和 `manus_metric` 深度。左右手使用同一型号各自的 profile 资源。保留上游的 retarget、Manus 绑定、A3 C++ SDK 1.1.15 原始限位和 Linux 原生驱动。

2. **TCP**：灵巧手不加 TCP，也不标定，遥操控制法兰（`--tcp-tool flange`）。
3. **跟随方式（仅 `dxhand` 末端）**：启动遥操后，双臂和手指自动同时跟随，不用握 Grip，也不受 Trigger 控制。自动 Home 完成且 PICO 数据新鲜时，双臂以当时的手柄位姿和 TCP 对齐后开始跟随。暂停（界面按钮或空格）、Home、退出、断流时，双臂按现有逻辑保持，手指同时保持；恢复时重新对齐。Manus 数据过期只影响手指。**法兰 / TacCap 末端（单独双臂遥操）的行为完全不变**，仍是握 Grip 跟随、松开保持。
4. **操作方式**：PICO 手柄固定在 Manus 手套上，操作者不握手柄；操作者站立；另一人在界面点「启动遥操 · 使能上半身」，同时启动双臂和双手。手腕位姿来自 PICO 手柄，属于 IK 部分，与本方案无关。

## 上游引入

| 内容 | 来源 | 处理 |
|---|---|---|
| Retargeting + Manus 绑定 + 驱动 + SDK 1.1.15 | `TsenseIV2C-main` | 按运行链路的 import 闭包复制子集到 `third_party/tsensei_retarget/`，保持原目录结构，**不修改上游源码** |
| Manus SDK 3.1.1 运行库 | `TsenseIV2C-main/vendor/manussdk/` | 专有二进制，不入库；setup 按 `tools/dxhand/manus-runtime.json` 校验库、链接和头文件，在 `.local/dxhand/vendor/manussdk/` 编译绑定 |

- 运行链路：`online_motion_track/manus_source.py`（Manus → 21 点）→ `pipeline_v3_2` / `src/hand_profiles.HandModelSession`（retarget）→ `a32/control.py`（`A32Control`，Ruckig 500 Hz）→ `src/a32_driver`（`A32Device` + `liba32_native`）。
- 不复制的部分：Web 服务与浏览器 UI、mediapipe/相机、mink/mujoco 碰撞、录制与动作回放、日志，以及预编译的 `bin/` 和 `build-linux/`。
- `THIRD_PARTY_NOTICES.md` 记录：来源目录、日期、所含 SDK 版本 1.1.15 （未打 CAD 限位补丁）和上游真机记录，以及省略了哪些内容。

## 架构

```
PICO ──> vr_client（现有 450–500 Hz 主循环，不做 I/O）
            │ stdin/stdout JSON 行（启动、follow on/off、心跳；状态 10 Hz 回传）
            ▼
         dxhand_service（独立进程，.local/dxhand/env，Py3.12）
            Manus Integrated（进程内）→ 21 点 → retarget（20 关节）
            → A32Control（Ruckig 500 Hz）→ A32Device（liba32_native）──USB CDC──> 左/右手
```

- **独立进程**：retarget 每帧要解 QP，依赖（scipy、daqp、ruckig、Manus 绑定）与 PICO 环境不同；放在独立进程里不会抢主循环的 GIL，崩溃也互相隔离。
- **左右手**：每只手一个 USB 口，各对应一个 `A32Device`。读到的 `hand_side` 与预期不一致时拒绝启动。允许只接一只手。
- **保持条件**：以下任一情况出现时手指保持当前位置：
  - Manus 数据超过 200 ms 未更新；
  - vr_client 心跳超过 200 ms；
  - `follow` 为 off。

  原因：SDK 没有主机看门狗，进程停发后电机会保持最后的指令。
- **启动**：使能，必要时回零，回零过程中手指会动。全部就绪后才接受 follow。
- **退出**：先保持当前位置，再下使能，并等到下使能确认。界面提示：手断电后会松开所抓物体。
- **本地锁文件**：防止两个进程同时打开同一只手。灵巧手不申请机器人端租约。

## 阶段 3 接入点（pink-ik merge 后）

- `scripts/pico4/fairino/vr_dxhand.py` 中的 `DxHandTeleop`，接口与 `GripperTeleop` 相同：`start / update(frame, hands, active) / snapshot / close / finish`。`update()` 只把 follow 状态和心跳写进管道，不等待。
- `vr_client.py`：
  - 新增 `--dxhand` 及其参数，与 `--grippers` 并列，二者互斥；
  - `operate()` 里与 grippers 一样调用；
  - 状态打印和 GUI IPC 增加 `dxhand`。
- `apps/workbench/teleop.py`：
  - 现有「遥操 TCP」下拉框与「TacCap 勾选」合并为「末端」下拉框：`flange / taccap / dxhand`。`dxhand` 固定使用法兰 TCP。
  - 增加达显面板：左手 / 右手启用、USB 口（默认自动）、Manus 校准文件（可选），状态行。
- 模型 Demo：可以选 dxhand，只显示法兰运动，不渲染手指。
- `vr_client.py`：`--dxhand` 时，双臂改为自动接合（替代 Grip 离合），只改 dxhand 分支，不动其他末端的离合逻辑。
- 测试：照 `test_vr_grippers.py`、`test_teleop_options.py` 补测试；flange / taccap 的行为不变（仍按 Grip 离合）。

## 分步

**阶段 1：引入 + 构建 + 离线验证（不连硬件）**
- 新增 `third_party/tsensei_retarget/`，更新 `THIRD_PARTY_NOTICES.md`。
- 新增 `tools/dxhand/setup_dxhand.py`，并在 `setup.sh` 增加 `dxhand` 子命令。setup 负责：
  - 建 `.local/dxhand/env`；
  - 恢复并校验 Manus 运行库；
  - 编译 Manus 绑定；
  - 编译 `liba32_native` 并运行上游协议与 Linux 传输离线测试。

  所有产物都放在 `.local/dxhand/`，不写进 `third_party/`。
- 离线验证：用合成 21 点跑 retarget。我们的入口和 main 的 Manus WebSession 原链路要逐值一致，同时测量单帧耗时。

**阶段 2：`dxhand_service` 独立台架（只连手 + 手套）**
- 新增 `scripts/dxhand/dxhand_service.py`：
  - 实现上面的进程、保持条件、心跳、退出逻辑；
  - 对外提供 JSON 行协议；
  - 提供 `--bench` 模式，不依赖 PICO，用键盘切换 follow。
- `run.sh dxhand` 入口。
- 用假设备和假 Manus 写测试，覆盖：
  - 心跳丢失、Manus 过期、单手；
  - 退出时下使能、`hand_side` 不符；
  - 协议非法输入。
- 台架实测（用户）：回零、跟随，以及拔 USB 或杀进程时手的行为。

**阶段 3**：merge `feat/pink-ik` 后按上一节接入。

**阶段 4：真机（用户在场）**
1. 先用法兰模式，双臂和双手低速试；
2. 再换 `fairino_qp`。
