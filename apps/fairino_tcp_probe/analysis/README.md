# TCP 模型与法兰分析

本目录包含独立 URDF 正解比较、法兰偏移校验、手腕绘图及数值记录。当前使用法兰偏移 `[-106,0,0] mm` 和双臂胸部安装变换。

仓库根目录执行：

```bash
.venv/bin/python apps/fairino_tcp_probe/analysis/model_audit.py
.venv/bin/python apps/fairino_tcp_probe/analysis/flange_check.py
bash run.sh diagnostics wrist
```

输入为 `models/` 内左右臂 URDF 与手腕网格，数学库在 `vendor/fairino/`。

结果：[模型审计](model_audit.json)、[法兰校验](flange_check.json)、[连续场景](flange_runtime_validated.json)、[TCP 图](tcp_audit.png)、[手腕图](wrist_geometry.png)。模型误差不等于实机定位精度。
