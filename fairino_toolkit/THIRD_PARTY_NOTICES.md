# 第三方资源

本包是用户已有文件的当前工作快照，不重新授予任何第三方授权。

- `src/lerobot` 来自本机 lerobot-xense；原许可证见 `vendor/lerobot_source/LICENSE`（如上游提供）。
- `vendor/fairino/libkine_FR_7Axis.so` 及头文件来自用户持有的法奥 SDK_V3.0.0；授权范围以厂家为准。
- PICO 的 APK、PC Service 和机器人 SDK 来自原项目记录；版本、下载地址和 SHA256 见 `scripts/pico4/assets.json`。
- `vendor/assets/platform-tools-latest-linux.zip` 为本机已下载的 Android Platform Tools 快照，校验见总清单。
- 厂家模型保留原文；模型中的已知差异见功能汇总和历史资料，不应整体替换实机 ROS 配置。

本包不包含操作系统镜像、厂家全部机器人运行时及 Python 的完整离线 wheel 仓库。历史资料按原文件保留，相关登录信息的使用范围不因打包而改变。新电脑依赖安装仍需联网；机器人端需要当前已部署的 ROS / HAL 服务。
