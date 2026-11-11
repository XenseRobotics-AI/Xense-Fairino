# Tron2 C++ QP IK

这是一个独立的 Tron2 双臂逆运动学项目，只包含：

- `models/tron2/robot_grasper.urdf` 及其 STL 网格；
- Pinocchio 模型加载、FK 和关节映射；
- C++ Pinocchio + eiquadprog 的双末端约束 QP IK；
- 左右臂分别控制的命令行和 Qt 滑动条界面；
- 可选 MeshCat 三维显示。

Python 只提供接口、CLI 和 Qt；FK、雅可比、mimic 展开、约束 QP 和迭代全部在 C++ 扩展中执行。

## 模型约定

原始 URDF 在 Pinocchio 中有 36 个配置变量，其中包含头部、夹爪驱动和夹爪联动关节。本项目在 IK 中只保留两条七轴手臂：

```text
left:  proximal_pitch, proximal_roll, proximal_yaw, elbow,
       wrist_yaw, wrist_pitch, wrist_roll
right: proximal_pitch, proximal_roll, proximal_yaw, elbow,
       wrist_yaw, wrist_pitch, wrist_roll
```

因此公开配置向量固定为 14 维，顺序为左臂 7 轴再右臂 7 轴。头部和夹爪在求解时固定，mimic 关节会在完整状态和 MeshCat 显示中自动回填。

默认末端帧为：

- 左臂：`wrist_roll_L_Link`
- 右臂：`wrist_roll_R_Link`

所有目标位姿均是相对于 `base_Link`（世界/基坐标系）的 `4x4` 齐次矩阵。Qt 中 XYZ 单位是米，关节角与 RPY 输入单位分别为度和弧度。

## 安装

推荐新建独立环境：

```bash
cd /home/xense/tron2/tron2-ik
mamba env create -f environment.yml
mamba activate tron2ik
```

`environment.yml` 已经以 editable 模式安装当前项目以及 Qt、MeshCat 和测试依赖。

如果要安装到已有环境：

```bash
python -m pip install -e .
```

该命令会同时安装核心、Qt、MeshCat 和测试依赖，并固定 Pinocchio 3.9.0
与其匹配的 urdfdom/tinyxml2 原生 ABI。请确保 `python` 和 `pip` 来自同一个
已激活环境；也可以直接使用该环境的绝对路径，例如
`/home/xense/miniforge3/envs/tron2/bin/python -m pip install -e .`。

启动命令会检测 `/opt/ros` 中不兼容的 Pinocchio Python/动态库路径，并自动以隔离环境重启。也可以手动运行：

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH python examples/dual_arm_qt.py
```

QP 后端固定为原生 C++ `eiquadprog`，不会调用 Python `qpsolvers`/`daqp`。

## Qt 滑动条界面

```bash
tron2-ik-qt
```

或直接运行：

```bash
python examples/dual_arm_qt.py
```

不启动 MeshCat：

```bash
python examples/dual_arm_qt.py --no-meshcat
```

界面包含：

- 左右臂各 7 个关节 FK 滑动条；
- 左右末端各自的 XYZ/RPY 目标滑动条；
- `Solve left`、`Solve right` 和 `Solve both`；
- 原生 C++ QP solver 状态；
- Reset、从当前 FK 写入 IK 目标以及动画显示。

求单臂时，另一侧目标会保持当前 FK 位姿，因此仍由同一个 14 维双臂 QP 连续求解。

## 命令行示例

用内置的小幅双臂目标做一次非零 C++ QP 自检：

```bash
tron2-ik
```

指定左右完整 XYZ/RPY 目标（米、弧度）：

```bash
python examples/dual_arm_ik.py \
  --left-target 0.03 0.18 -0.54 0 0 0 \
  --right-target 0.03 -0.18 -0.54 0 0 0
```

查看全部参数：

```bash
tron2-ik --help
tron2-ik-qt --help
```

## Python API

```python
import numpy as np
from tron2_ik import IKOptions, Tron2QPModel

model = Tron2QPModel()
q_seed = np.zeros(14)

left_target = model.forward_kinematics(q_seed, side="left")
right_target = model.forward_kinematics(q_seed, side="right")
left_target[0, 3] += 0.02
right_target[0, 3] += 0.02

result = model.solve_ik(
    left_target,
    right_target,
    q_seed,
    IKOptions(qp_solver="eiquadprog", tolerance=1e-4),
)

print(result.success, result.q, result.as_dict())
```

独立目标可通过批量接口减少 Python/C++ 往返；左右目标形状为
`(B, 4, 4)`，初值形状为 `(B, 14)`，返回按输入顺序排列的 `IKResult` 列表：

```python
results = model.solve_ik_batch(
    np.stack([left_target, left_target]),
    np.stack([right_target, right_target]),
    np.zeros((2, 14)),
    IKOptions(tolerance=1e-4),
)
```

每个目标仍使用完整约束 QP，共用一次配置转换和模型锁，不会自动沿用上一目标的解。
`result.full_q` 在同次原生求解中生成，包含头部、夹爪与 mimic 关节的完整 36 维状态。
批量结果的 `solve_time_ms` 是各目标的原生求解时间，不含共用配置与 Python 转换；
评估批量吞吐时应测量整个 `solve_ik_batch` 调用。

每轮在 C++ 中解一个带关节位置、速度和立柱线性化安全约束的 QP，再用 Pinocchio 更新流形状态。较远或不可达的目标可能返回 `success=False`；连续拖动时应始终把上一帧结果作为下一帧 seed。

## 测试

```bash
pytest -q
QT_QPA_PLATFORM=offscreen pytest -q tests/test_qt.py
```

测试覆盖模型资源、14/36 维映射、双臂 FK→C++ QP IK、mimic 回填以及 Qt 离屏构造。

## 目录

```text
src/tron2_ik/                 Python 包
src/tron2_ik/assets/tron2/    可随 wheel 安装的 URDF/STL
models/tron2/                 指向包内模型的源码树快捷路径
examples/                     CLI 与 Qt 启动脚本
tests/                        自动测试
```
