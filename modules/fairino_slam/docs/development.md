# 开发指南

## 目录职责

`docs` 是 documentation 的缩写，存放持续维护的说明，不参与机器人运行。`reports` 存放某个日期的验证事实，不把旧测试结果当成当前保证。

`src/fairino_slam` 是 Python 导入包：外层仓库可以改名，内部包名则是程序接口。采用 src 布局可避免把仓库根目录中的测试、脚本和文档误当作安装包的一部分。依据 [PyPA 的 src 布局说明](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/)。

`config/` 是唯一参数源码，电脑端、配置生成器和部署工具都通过统一读取层使用它。构建钩子 `setup.py` 将配置和 ros2 插件源码复制到 wheel 的包资源目录；editable 安装直接读取根配置，不维护手工副本。模型及 SLAM 构建资源仍留在 `resources/`。

本工程包含电脑端 Python 应用和 `ros2/fairino_lookahead_controller` 的实际 Nav2 C++ 插件。ROS 包可用 colcon 构建；电脑端核心测试只需 C++17 编译器，无需 ROS。机器人已有 ROS/厂家软件依赖见 deployment.md。

## 环境与检查

安装和运行入口分别为 `scripts/setup.sh`、`scripts/run.sh`。`scripts/check.sh` 负责 Shell 语法、配置和测试检查；加上 `--package` 时调用 `scripts/check_package.py`，验证分发包的完整性和独立运行。CI 使用同一入口。

```bash
bash scripts/setup.sh dev
bash scripts/check.sh
bash scripts/check.sh --package
```

三个入口共用 `scripts/python_env.sh`：显式 `FAIRINO_SLAM_PYTHON` 优先，其次是已激活 Conda 的 Python，再使用工程 `.venv`。Conda 和指定解释器模式不创建或修复 `.venv`。环境存在但无法执行时明确失败，避免意外安装到别的环境；无现成环境时 `setup.sh` 才创建 `.venv`，可通过 `FAIRINO_SLAM_SYSTEM_PYTHON=/path/to/python3` 指定创建它的 Python 3.10+。

通过 SSH 开发时使用实际仓库 `/home/xense/fairino_slam`，例如：

```bash
cd /home/xense/fairino_slam
/home/xense/miniconda3/bin/conda run --no-capture-output -n env_fairino bash scripts/run.sh check
/home/xense/miniconda3/bin/conda run --no-capture-output -n env_fairino bash scripts/check.sh
```

`setup.sh gui` 会使用同一解释器检查 GTK3、gi 和 Cairo；系统 apt 安装的 Python GUI 包不保证对 Conda 可见。跨电脑复制源码时排除 `.venv`。

开发依赖在 pyproject.toml 的 dev extra 中声明。检查脚本不连接机器人。C++ 控制核心通过 `scripts/check_controller.sh` 用系统编译器直接构建；完整 ROS 插件单独在 Humble CI 作业验证。macOS 如果默认 SDK 与链接器不兼容，可用 `SDKROOT` 指定本机兼容 SDK。完整分发检查在临时目录执行构建，验证源码包包含完整测试与必要文档，再从解压出的源码包和 wheel 运行检查；临时产物自动清理。

CI 使用同一检查脚本，配置 Python 3.10 和 3.12。GitHub 托管执行尚需仓库推送后触发；本地通过不能冒充所有版本的 CI 已通过。

## 编码与命名

- Python 文件、函数和变量使用 snake_case，类使用 PascalCase，常量使用 UPPER_SNAKE_CASE。
- 缩进使用 4 空格，UTF-8 和 LF；编辑器约定见 .editorconfig。
- 包内部使用明确的 fairino_slam 导入或相对导入，不搜索原 toolkit 或父目录补依赖。
- 普通库导入不连接机器人、不使能、不执行部署；操作由明确的 CLI/GUI 动作触发。
- `_remote.py` 表示发送到机器人独立执行的脚本；它不能假定机器人安装了电脑端包。
- `navigation.launch.py` 保留 ROS 启动文件惯例；YAML、XML 和模型是运行资源。
- 新文件按职责放入现有模块；不为单个辅助函数新增并列顶层模块。

## 修改与验证边界

先运行受影响模块的离线测试，再执行统一检查。修改部署路径时必须验证生成 payload，以及机器人端平铺文件的导入闭包。修改窗口会话时检查建图与导航互斥、焦点丢失、连接中取消和退出等待。

参数修改入口见 [config/README.md](../config/README.md)。`configuration.py` 和 `navigation/settings.py` 校验输入；`navigation/profile.py` 只负责将共享速度、加速度、周期和模型轮廓注入模板。生成文件写 `.local/generated/nav2.yaml`，不提交 Git。新增参数需同步配置校验和有意义的回归测试。

`chassis/source.py` 将经过校验的参数快照嵌入机器人端源码，覆盖内存 exec 和平铺导入；远端无需本机包或配置目录。控制连接使用本进程加载的快照；改文件后需要重启电脑端程序，部署则读取部署时的配置。

协议名如 `/fairino_keyboard/cmd_vel`、`.fairino_keyboard.lock` 和已有 systemd 服务名属于机器人接口，不能当成普通文件重命名。实机部署、运动验证与离线测试分别记录，不把已有离线通过当成硬件通过。

## 分发

wheel 只包含运行包和所需资源。源码包通过 MANIFEST.in 包含完整 config、tests、docs、scripts、入口、第三方说明和 reports 中的文本记录，保持文档链接完整；运行日志及虚拟环境不进入分发包。

已有代码未明确授权新的开源许可证，本次工程规范化不擅自补 MIT/Apache 等许可。第三方来源继续保留原说明。
