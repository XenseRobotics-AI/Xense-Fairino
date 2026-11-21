# SDK 来源

此目录复制自本机 `lerobot-xensehand/third_party/manussdk`，其 SDK 子仓库提交为 `8a5ef6b1f370ad369643f54084aac8fba86955b6`。该提交标识 Xense 维护的 Python 绑定和打包结构，厂商运行库版本为 Manus 3.1.1。

保留上游已跟踪的源文件、示例、构建配置、头文件、Linux x86_64 动态库、Git LFS 属性与许可证。按用户要求补入参考仓库 `calibration/` 中的六个 `.mcal` 文件，保留原名并校验内容一致。未复制 Git 元数据、日志和构建产物。适配代码位于 `online_control/online_motion_track/manus_source.py`，构建入口位于 `scripts/build_manus.sh`。

厂商许可证见 [license/license.txt](license/license.txt)。标定文件的选择和启动参数见 [Manus 接入说明](../../docs/manus.md)。

本目录随后随 Manus 输入适配代码迁入 `TsenseIV2C-main`。构建产物在目标仓库 track 环境重新生成，不包含源仓库的虚拟环境或构建缓存。
