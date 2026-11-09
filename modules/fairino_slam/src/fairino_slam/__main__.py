"""Lazy command dispatch: help and offline checks never contact the robot."""
import importlib
import sys

COMMANDS = {
    "map": ("mapping.viewer", "地图、定位与导航窗口"),
    "mapping": ("mapping.client", "建图、地图库和 AMCL 定位操作"),
    "navigation": ("navigation.client", "导航准备、执行底盘、规划与目标"),
    "faults": ("maintenance.faults", "底盘/上身故障检查与单次复位"),
    "recover": ("maintenance.recovery", "检查或一键恢复底盘接入，不自动使能"),
    "chassis": ("chassis.client", "底盘只读检查或不使能的接口检查"),
    "network": ("connection.network", "本机到机器人网段的路由检查"),
    "inspect": ("diagnostics.inspect_robot", "机器人硬件、雷达和控制器只读检查"),
    "record-odometry": ("diagnostics.record_odometry", "只读记录里程计、TF 和匹配结果"),
    "check-link": ("diagnostics.check_control_link", "SSH/ROS 只读心跳诊断"),
    "check-controller": ("diagnostics.check_navigation_controller", "本机隔离 ROS 域中的控制器验证"),
    "build-config": ("deployment.build_navigation", "生成或检查导航配置（离线）"),
    "check": ("diagnostics.offline", "检查包、配置和远端部署文件（离线）"),
    "test": ("diagnostics.offline", "运行离线回归测试"),
}
DEPLOY = {
    "control": "install_control_stack",
    "mapping": "install_map_mode",
    "slam": "install_mapping_runtime",
    "navigation": "install_navigation",
    "priority": "install_session_priority",
}


def help_text():
    rows = ["用法：fairino-slam <命令> [参数] / bash scripts/run.sh <命令> [参数]", ""]
    rows += [f"  {name:18} {description}" for name, (_, description) in COMMANDS.items()]
    rows += ["  deploy <组件>      control | mapping | slam | navigation | priority", "",
             "每个子命令支持 --help。check/test/build-config 为离线操作。",
             "navigation drive 会回正并使能底盘；goal 才下发导航目标。",
             "其余在线命令按各自帮助说明执行。"]
    return "\n".join(rows)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in ("help", "-h", "--help"):
        print(help_text())
        return 0
    command = args.pop(0)
    if command == "deploy":
        if not args or args[0] in ("help", "-h", "--help"):
            print("用法：bash scripts/run.sh deploy control|mapping|slam|navigation|priority [--help]")
            return 0
        component = args.pop(0)
        if component not in DEPLOY:
            print("未知部署组件：" + component, file=sys.stderr)
            return 2
        module = "deployment." + DEPLOY[component]
    elif command in COMMANDS:
        module = COMMANDS[command][0]
        if command == "test":
            args.insert(0, "--tests")
    else:
        print("未知命令：" + command + "\n" + help_text(), file=sys.stderr)
        return 2
    sys.argv = ["fairino-slam " + command, *args]
    return importlib.import_module("fairino_slam." + module).main()


if __name__ == "__main__":
    raise SystemExit(main())
