# 当前控制工具包

这是 2026-10-14 的可移植源码仓库。安装资源按清单下载。完整说明见 [功能汇总](../文档/功能汇总.md)。

```bash
bash run.sh help
bash setup.sh base
bash setup.sh pico
bash run.sh test
```

常用入口：`base` 底盘、`joints` 单轴、`body` 身体联动、`tcp` 末端键控、`lidar` 雷达显示、`pico` 头显安装/接收、`vr` 面对面遥操、`errors` 故障检查/清除。

`vr` 默认 450 Hz 插值/IK/下发、500 Hz 接收轮询、100 Hz 状态回传、12 ms 输入缓冲。使用 `pico fairino` 原入口时，源码默认仍为 90/200/100 Hz。当前是先 TCP 插值再 IK；机器人不再添加旧的 30 ms 关节追赶段。

`bash run.sh errors --execute` 使用本包新版上下身清错脚本，不使能或运动；旧 `reset-upper --execute-reset` 仅作兼容。

`evidence/` 下的扫描/迁移脚本是历史诊断代码，部分会更改配置；不是日常运行入口。本包安装流程不会执行这些脚本。
