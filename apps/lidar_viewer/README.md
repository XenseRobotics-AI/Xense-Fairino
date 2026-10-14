# 双雷达显示

模块位置 `apps/lidar_viewer/`。仓库根目录运行：

```bash
bash run.sh lidar
```

`viewer.py` 为电脑界面，`remote_stream.py` 经 SSH 在底盘读取 ROS 扫描。默认连接 `fairino@192.168.58.3`，可用 `--host`、`--user` 改端点。密码在终端输入。

支持显示半径调节和画面暂停，两幅图分别使用自己的扫描坐标系。关闭界面结束显示连接，底盘雷达服务继续运行。

启动器优先使用基础环境，缺少 Tk 时回退到项目 PICO 环境；`bash setup.sh all` 准备运行依赖。样例与图片见 [雷达与模型](../../docs/雷达与模型.md)。
