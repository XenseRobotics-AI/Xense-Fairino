# 20 轴单关节控制

模块 `apps/fairino_upper_probe/`。仓库根目录执行：

```bash
bash run.sh joints --group left
bash run.sh joints --group right
bash run.sh joints --group body
```

连接上半身 `192.168.58.1`、ROS Domain 0，使能全部 J1–J20 并保持当前角度。左臂 J1–J7，右臂 J8–J14；J15 头部，J16 腰旋转，J17 侧倾，J18–J20 折叠机构。一次只移动一个关节，其余 19 轴保持。

数字键选轴，左右箭头 ±0.5°，支持绝对角度与计划速度输入；空格中止并保持，Esc/关闭退出并请求全部下使能。分页只改变选择，不重复使能。

本入口取消 ±3°相对范围、固定速度上限、实测超速退出及路径跟踪误差中止。急停、硬件故障、反馈/SSH 超时、终点和保持检查继续工作。

`upper_keyboard.py` 为界面/SSH，`probe_core.py` 为纯逻辑，`robot_probe.py` 为共享 ROS 生命周期。[joint_mapping.json](joint_mapping.json) 保存辨认记录；运行日志生成于 `apps/fairino_upper_probe/logs/`。

```bash
bash run.sh joints --check
bash run.sh joints --prepare-check
bash run.sh joints --demo
bash run.sh test
```

身体联动用 `bash run.sh body`，见 [身体说明](body_control/README.md)。开机检查见 [故障排查](../../docs/故障排查.md)。
