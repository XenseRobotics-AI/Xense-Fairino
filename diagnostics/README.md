# 离线诊断

仓库根目录执行；`bash setup.sh all` 准备依赖，输入模型及样例已包含在项目中。

```bash
bash run.sh diagnostics geometry
bash run.sh diagnostics lidar
bash run.sh diagnostics wrist
bash run.sh diagnostics fault-example
```

| 命令 | 代码 | 输入 |
|---|---|---|
| geometry | [model_geometry/](lidar_diagnostics/model_geometry/) | `models/meshes/base_link.STL` 与圆环数据 |
| lidar | [alignment/](lidar_diagnostics/alignment/) | `data/lidar/scans-sample.json.gz` |
| wrist | [plot_wrist.py](../apps/fairino_tcp_probe/analysis/plot_wrist.py) | `models/meshes/` 中的手腕网格 |
| fault-example | 项目 JSON | [故障样例](../data/diagnostics/startup-fault30-reset.json) |

这些离线入口不会改变机器人服务、参数或电机状态。生成物位于相应诊断目录，由 Git 忽略规则管理。

[battery_diagnostics/](battery_diagnostics/) 为电池串口/协议排查代码，部分需在机器人串口所在环境运行，尚未形成确认可用的电量 API。`lidar_diagnostics/integrate_front_back_20261008.py` 为底盘服务维护脚本，不属于上述离线入口。
