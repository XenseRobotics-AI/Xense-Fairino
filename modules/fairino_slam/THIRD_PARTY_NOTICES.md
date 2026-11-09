# 第三方资源与来源

- SLAM Toolbox 独立运行程序基于官方 **2.6.10**。本工程保留源码文件 SHA-256 清单、选帧补丁和 CMake；部署时从上游获取源码及 LICENSE，并随独立程序安装上游 LICENSE。清单见 `src/fairino_slam/resources/slam_toolbox/sources.json`。本次迁移未下载或重新编译上游源码。
- Nav2、ROS 2、robot_localization、Karto 和厂家控制器由机器人已有环境提供，不在本工程重新分发其完整源码或二进制。
- `amr_8dof.urdf` 来自原工程已保存的厂家运行模型快照，仅含所需基本几何体，来源与校验见模型 manifest。迁移不改变原有模型权利。
- pexpect、PyYAML、NumPy、Matplotlib 及其依赖按各自许可证使用，由安装工具获取。
- 未迁入 PICO、Lerobot、TacCap 或其资源。本次整理不为原工程代码新授予许可证；第三方原有许可保持有效。
