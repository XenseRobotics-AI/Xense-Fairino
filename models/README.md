# 运行与诊断模型

| 文件 | 用途 |
|---|---|
| [left_arm.urdf](left_arm.urdf)、[right_arm.urdf](right_arm.urdf) | 双臂独立模型验证 |
| [whole_body_20260731.urdf](whole_body_20260731.urdf) | 胸部安装与折叠机构模型依据 |
| [base_link.STL](meshes/base_link.STL) | 底盘/雷达机械位置分析 |
| [wrist1_Link.STL](meshes/wrist1_Link.STL)、[wrist2_Link.STL](meshes/wrist2_Link.STL) | 腕心和法兰偏移绘图 |
| [manifest.json](manifest.json) | 大小与 SHA-256 |

URDF 保持原始字节以支持哈希校验。全身控制编号、物理限位与动力学仍须结合实际设备确认。见 [雷达与模型](../docs/雷达与模型.md)。
