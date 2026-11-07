# Fairino SLAM

法奥四舵轮机器人建图、定位与二维导航工程。电脑通过 SSH 调用机器人端 ROS 2 Humble，提供建图键控、地图管理、AMCL 定位、Nav2 导航及故障处理。

## 工程结构

本工程采用 Python 标准 `src` 布局。外层 `fairino_slam/` 是仓库，`src/fairino_slam/` 是 Python 包；两者职责不同，不是重复源码。包名用于 `import fairino_slam` 和安装后的命令入口。

```text
fairino_slam/                 # 仓库根目录，可自由改名
├── pyproject.toml           # 构建配置、运行依赖、开发依赖和命令入口
├── config/                  # 统一调参入口：底盘、导航、SLAM、AMCL
├── src/fairino_slam/        # Python 源码、模型及 SLAM 构建资源
├── ros2/                    # 实际的 Nav2 预瞄控制器插件源码
├── tests/                   # 按功能模块组织的离线测试
├── docs/                    # 持续维护的架构、开发、部署和操作文档
├── scripts/                 # 安装、运行与开发检查入口
├── reports/                 # 有日期的验证与迁移记录
└── .github/workflows/       # 自动化检查配置
```

| 源码模块 | 职责 |
|---|---|
| `mapping/` | SLAM 会话、地图管理、AMCL、地图显示与远端观察 |
| `navigation/` | 规划配置、生命周期、目标管理与速度门控 |
| `chassis/`、`odometry/` | 建图/导航共用底盘执行、轮速适配与守护 |
| `connection/`、`maintenance/` | SSH、路由检查及故障处理 |
| `deployment/`、`diagnostics/` | 机器人部署、诊断与离线校验 |
| `ui/` | 建图键控、界面会话及 GTK 支持 |
| `resources/` | URDF、SLAM 源码清单与补丁；配置由根 config/ 随包收录 |

以上源码模块均在 `src/fairino_slam/`。构建钩子 `setup.py` 仅负责自动收录根 `config/` 和 `ros2/`，无需直接运行。`.venv/`、`.local/`、`logs/` 是本机生成内容，不提交 Git；`build/`、`dist/` 是可再生成的构建产物。开发检查在临时目录构建，不向工程根目录留下 `build/`。

## 脚本入口

所有 Shell 入口集中在 `scripts/`。以下命令从仓库根目录执行；也可在任意目录通过脚本的绝对路径调用，工程路径由脚本自动定位。

| 脚本 | 用途 | 常用命令 |
|---|---|---|
| `scripts/setup.sh` | 使用 Conda 或项目虚拟环境，安装依赖 | `bash scripts/setup.sh gui` |
| `scripts/run.sh` | 运行建图、导航、故障处理等功能 | `bash scripts/run.sh map` |
| `scripts/check.sh` | 开发和 CI 的统一离线检查 | `bash scripts/check.sh --package` |
| `scripts/check_package.py` | 构建并验证源码包和 wheel，由 `check.sh --package` 调用 | 通常无需直接运行 |

## 开始使用

需要 Python 3.10+ 和系统 SSH；电脑无需安装 ROS。

已有 Conda 环境时直接复用，不创建或访问工程 `.venv`：

```bash
conda activate env_fairino
bash scripts/setup.sh cli
bash scripts/run.sh check
```

安装、运行和检查脚本统一按 `FAIRINO_SLAM_PYTHON` → 已激活 Conda → 工程 `.venv` 选择解释器；无以上环境时，安装脚本创建 `.venv`，运行和检查脚本使用 PATH 中的 `python3`。指定的环境不可用时明确报错，不回退到其他环境。`scripts/python_env.sh` 是这些入口共用的辅助文件，不需直接运行。

没有现成环境时：

```bash
bash scripts/setup.sh cli
bash scripts/run.sh help
bash scripts/run.sh check
bash scripts/run.sh test
```

完整界面使用 GTK3。Conda 环境中的 `gi` 由 PyGObject 提供，绘图后端还需要 PyCairo 和 GTK3。先在目标环境补齐依赖，再检查 GUI 安装：

```bash
conda activate env_fairino
CONDA_AGGRESSIVE_UPDATE_PACKAGES= conda install --freeze-installed -c conda-forge pygobject pycairo gtk3
bash scripts/setup.sh gui
```

可在没有显示器的 SSH 会话中验证依赖，不创建窗口、不连接机器人：

```bash
conda run -n env_fairino python -c 'import gi, cairo; gi.require_version("Gtk", "3.0"); gi.require_foreign("cairo"); from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg; print("GTK3Agg imports passed")'
```

以下 apt 安装方式适用于 Ubuntu 系统 Python；采用此方式时先退出 Conda，避免把依赖装到不同环境：

```bash
sudo apt install python3-venv python3-gi python3-gi-cairo gir1.2-gtk-3.0 openssh-client fonts-noto-cjk
bash scripts/setup.sh gui
bash scripts/run.sh map
```

`scripts/setup.sh` 只安装电脑环境。`check`、`test` 完全离线；`map` 会连接机器人，点击启动建图底盘或接入导航底盘才回正并使能。可用 `FAIRINO_SLAM_PYTHON` 指定解释器，`FAIRINO_SLAM_LOG_DIR` 指定日志目录。复制工程时不要复制 `.venv`；在目标电脑复用 Conda 或重建虚拟环境。已有残缺 `.venv` 不会被脚本覆盖，激活 Conda 即可绕过。

## 恢复底盘接入

地图窗口的 **恢复底盘接入** 可统一处理退出后的遗留临时控制器、可复位底盘故障和下使能确认。它先结束本窗口的控制会话，再检查并执行一次恢复；成功后需手动重新接入和初始化定位，不会自动继续旧目标。建图中请先保存地图。

命令行默认只读检查：`bash scripts/run.sh recover`；加 `--execute` 才执行恢复。日志位于 `logs/recovery/`，详细流程和失败原因见 [故障处理](docs/troubleshooting.md#一键恢复底盘接入)。更新代码后重开面板即可加载此功能，无需为这个入口重新部署导航。

## 开发

```bash
bash scripts/setup.sh dev
bash scripts/check.sh
bash scripts/check.sh --package
```

普通检查包括配置、Python 测试、C++ 控制/碰撞核心编译测试和入口语法（需要 C++17 编译器）；`--package` 还在临时目录构建源码包和 wheel，从分发包重新运行检查，防止缺文件或误用本地源码。安装开发依赖后，检查过程不需要网络、ROS 或机器人。

完整流程、命名与依赖边界见 [开发指南](docs/development.md)。

## 修改参数

统一编辑根目录 [config/](config/README.md)：速度、加速度改 `chassis.yaml`；导航频率、容差、避障改 `navigation.yaml`；SLAM 和定位分别改 `mapping.yaml`、`localization.yaml`。无需为这些参数修改 Python 代码。

```bash
bash scripts/run.sh check          # 校验参数与部署文件，不连接机器人
bash scripts/run.sh build-config   # 生成 .local/generated/nav2.yaml 供查看
```

当前默认速度为平移 0.10 m/s、旋转 0.20 rad/s。运行中的面板和机器人服务不热更新配置；修改后的生效步骤见 [参数配置说明](config/README.md)。

## 文档与验证

- [架构与模块职责](docs/architecture.md)
- [连接、建图、定位和导航操作](docs/workflow.md)
- [预瞄跟随与朝向策略](docs/navigation.md)
- [机器人部署](docs/deployment.md)
- [故障处理](docs/troubleshooting.md)
- [历史验证记录](reports/README.md)
- [第三方来源与许可说明](THIRD_PARTY_NOTICES.md)

当前导航使用 SmacPlanner2D + 全向预瞄控制器：当前配置为 0.20 m 欧氏预瞄距离、10 Hz、保持任务开始时朝向；也支持配置为 `path_tangent`，沿途跟随路径方向并在终点达到用户输入的朝向。通过位置/朝向误差生成连续速度，并保留速度保护与足迹碰撞检查。底盘轮廓不包含上半身动态投影；没有 ROS 的电脑可编译验证独立控制核心，完整 ROS 插件加载和实车效果另行验证。
