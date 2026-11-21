# 第三方资源与许可

- `src/lerobot/` 许可见 [LICENSE](vendor/lerobot_source/LICENSE)。
- `vendor/fairino/` 为法奥数学库及头文件，遵守原厂许可。
- `umi/vendor/TacCap-Gripper/` 为 XenseRobotics-AI/TacCap-Gripper 0.4.3 源码快照，revision `49d6a6ba587f13c97e145725034360a44d590013`，保留上游 [Apache-2.0 许可](umi/vendor/TacCap-Gripper/LICENSE)。
- PICO SDK 与绑定位于 `src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/`，保留源文件许可。
- PC Service/APK 版本、URL 与校验见 [assets.json](scripts/pico4/assets.json)，安装到项目 `.local/pico4/`。
- ADB 来自 Google 官方 Android Platform Tools。
- 雷达图来源见 [sources.json](docs/images/sources.json)，模型校验见 [manifest.json](models/manifest.json)。
- `third_party/fairino_ik/` 是 git submodule，指向内部仓库 `XenseRobotics-AI/fairino_ik` 的 `feat/pinocchio-4.1.0` 分支，固定在提交 `09a5674cc7353f005bf090fdeb54069952b954b7`。该提交未附 LICENSE 文件。

第三方资源原有许可不因项目集成而改变。

## 达显灵巧手

`third_party/tsensei_retarget/` 的上游是 `05_DXHand/TsenseIV2C-main`，提交
`cf42047e7dd7e864538b820245aa1ce557c2f50e`。服务使用 `TsenseI_v3_2` 的左右手
URDF、21 点标定文件和关节行程映射，输入为 `manus`（`manus_sdk`），深度来源为
`manus_metric`。上游源码和模型文件逐字节复制，目录内的 Git 换行规则由本仓库维护。

上游 2026-11-21 的 `logs/control_sessions/` 中，右手会话 `162842_608456` 使用
`collision_mode=off`，控制器接受了 6,799 帧，并记录了 USB 下行和遥测。
该会话的固件未报告有效手别，右手身份依据现场验证和会话配置。
左手会话 `164821_578837` 接受了 2,575 帧，固件有效报告 `hand_side=2`。
这些是上游记录。本仓库的迁移验证使用离线求解、模拟手套和伪终端。

A3 C++ SDK 为 1.1.15，源版本 `fd676e5a3296e5e3ada2734d290acc443206e000`，
使用上游原版行程换算和限位，未应用客户外壳 CAD 限位补丁。
[SDK 来源及原厂哈希](third_party/tsensei_retarget/vendor/a3-cpp-sdk/sdk-source.json) · [SDK 许可证](third_party/tsensei_retarget/vendor/a3-cpp-sdk/LICENSE)

Manus SDK 为 3.1.1。setup 从只读上游的 `vendor/manussdk/` 提取 Linux x86_64
运行库、SONAME 链接和头文件，逐项校验后在 `.local/dxhand/` 构建绑定。
运行库不随仓库分发。[Manus 来源](third_party/tsensei_retarget/vendor/manussdk/UPSTREAM.md) · [许可](third_party/tsensei_retarget/vendor/manussdk/license/license.txt) · [安装清单与 SHA-256](tools/dxhand/manus-runtime.json)

省略其他型号的资源、网格、Web 服务与前端、相机与 MediaPipe 推理、Mink/MuJoCo、
回放入口、日志与录制数据、专有二进制、压缩包、虚拟环境和构建产物。
控制器及管线强制导入的录制辅助类、坐标变换函数和 A3.0 共享模块随源码保留。
服务使用碰撞模式 `off`，不启动录制。离线构建测试包括 A3 协议和 Linux 传输；
依赖 Web bridge 示例的命令测试随该示例省略。
