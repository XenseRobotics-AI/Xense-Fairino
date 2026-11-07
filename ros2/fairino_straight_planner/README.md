# Fairino straight-first planner

这是 ROS 2 Humble 的 Nav2 全局规划插件。它组合已有的
`nav2_smac_planner/SmacPlanner2D`，不复制或修改 Nav2 源码，也不发送底盘速度。

## 规划行为

`GridBased.straight_line_enabled: true` 时：

1. 读取当前全局代价地图。起点到目标之间整条线可安全通过，就直接采用直线。
2. 直线被阻挡时，调用原 SmacPlanner2D 搜索绕行路径。
3. 对 Smac 相邻对角栅格连线的擦角情况，只在可以验证安全时补一个轴向折点；
   两条子线段都必须通过完整碰撞检查，否则规划失败。
   随后从当前转折点连接最远可见的后续点，删除不必要的转折。
   这是保守的可见性简化，不保证全局最少转弯或数学最短路径。
4. 按不大于地图分辨率的间距重新采样，供现有预瞄控制器跟随。
5. 返回前读取最新地图快照，再检查全部输出线段；若已被新障碍阻挡则规划失败。

每条完整线段采用 supercover 栅格检查，检查穿越的格子以及碰到角点、
沿栅格边界时的相邻格子。代价 253、254、未知 255 和越界均不可通行。
已知障碍继续使用全局代价地图原有的膨胀层和碰撞轮廓，不降低安全半径。
未知区域和地图外边界另按全局轮廓内切圆半径保护，避免只检查机器人中心。
本仓库的两种朝向策略仍由既有配置生成相应全局轮廓。

启用时需要有效的全局轮廓、开启的 InflationLayer，且膨胀半径不小于轮廓
内切圆半径。必须在已膨胀的 master costmap 上使用此插件。
启用时终点严格保持请求的位置和四元数；目标处不可通行时不接受附近替代点。
中间姿态按各直线段方向生成；保持当前朝向与沿路径朝向仍由控制器选择，
终点使用调用方指定的朝向。短于一个栅格的路径和原地调整朝向也保持精确目标。

`GridBased.straight_line_enabled: false` 时，直接调用并返回原 SmacPlanner2D：
不添加直达优先、简化、重新采样、精确端点或额外碰撞/时间限制。
因此可通过这个开关恢复本次修改前的规划行为。其他 Smac 参数使用同一个
`GridBased` 命名空间，保持原有含义。

## 参数与约束

```yaml
planner_server:
  ros__parameters:
    planner_plugins: [GridBased]
    GridBased:
      plugin: fairino_straight_planner::StraightLinePlanner
      straight_line_enabled: true
      # 以下仍由原 SmacPlanner2D 使用。
      max_planning_time: 2.0
      tolerance: 0.05
      allow_unknown: false
      use_final_approach_orientation: false
      smoother:
        max_iterations: 0
```

启用优化时，`max_planning_time` 同时覆盖地图准备、搜索、简化和验证。
按实际安装的 Nav2 接口自动兼容两参数与带取消回调的三参数 createPlan；
不根据 ROS_DISTRO 推断接口（厂商工作区可在 Humble 上回移新接口）。
三参数接口把取消回调传给原 Smac，新直线检查、简化、采样和输出循环也检查取消。
关闭优化时仍直接返回原 Smac 结果，并保留其取消行为。原两参数接口无法中途
传入取消；原 Smac 内部互斥锁等待也无法由包装层强制中断。搜索返回后若已超过
总预算，包装层拒绝输出路径。支持规划专用异常头时使用 PlannerCancelled，
不把取消转换成普通的规划失败。
包装层不会持有代价地图锁调用 Smac，避免与 Smac 自身锁顺序冲突。

地图快照只能描述读取时刻的障碍；规划返回后出现的动态障碍仍由现有局部
碰撞检查、停止保护和全局重规划处理。直线优先不会主动避开所有低于 253
的软代价，因此可能比原 Smac 路径更靠近允许通行区的边缘。需要更多间距时
应调整既有碰撞轮廓/安全余量，而不能关闭碰撞检查。

## 离线验证

`test/test_core.cpp` 是不依赖 ROS 的几何测试，`test/test_planner_api.cpp`
覆盖两种 API 的编译分派、取消回调及原结果/异常透传，均可用 C++17 编译器运行。
`test/test_plugin.cpp` 验证 ROS 插件、Smac 回退、开关、目标朝向和安全失败；
只能在装有 ROS 2 Humble/Nav2 的隔离环境运行。

```sh
colcon build --packages-select fairino_straight_planner --cmake-args -DBUILD_TESTING=ON
. install/setup.sh
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=193 colcon test --packages-select fairino_straight_planner
colcon test-result --verbose
ros2 run fairino_straight_planner plugin_check
```

`plugin_check` 仅通过 pluginlib 构造两个插件，验证本插件及 Smac 依赖可加载；
不调用 ROS 初始化，不创建节点、服务、代价地图或运动接口。
