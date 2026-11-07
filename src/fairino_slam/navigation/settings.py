"""Validate the supported Nav2 parameter template before deployment.

The schema records types and field names, not a second copy of parameter values.
Derived speed/geometry fields are deliberately absent and rejected on input.
"""
import copy
import math

from .runtime_policy import COMMAND_TIMEOUT


_FIELDS = dict(line.rsplit(" ", 1) for line in """
version integer
geometry/footprint_padding number
geometry/minimum_inflation_radius number
geometry/inflation_clearance number
nav2/planner_server/ros__parameters/use_sim_time bool
nav2/planner_server/ros__parameters/expected_planner_frequency number
nav2/planner_server/ros__parameters/planner_plugins texts
nav2/planner_server/ros__parameters/GridBased/plugin text
nav2/planner_server/ros__parameters/GridBased/straight_line_enabled bool
nav2/planner_server/ros__parameters/GridBased/tolerance number
nav2/planner_server/ros__parameters/GridBased/allow_unknown bool
nav2/planner_server/ros__parameters/GridBased/downsample_costmap bool
nav2/planner_server/ros__parameters/GridBased/max_planning_time number
nav2/planner_server/ros__parameters/GridBased/cost_travel_multiplier number
nav2/planner_server/ros__parameters/GridBased/use_final_approach_orientation bool
nav2/planner_server/ros__parameters/GridBased/smoother/max_iterations integer
nav2/controller_server/ros__parameters/use_sim_time bool
nav2/controller_server/ros__parameters/controller_frequency number
nav2/controller_server/ros__parameters/odom_topic text
nav2/controller_server/ros__parameters/min_x_velocity_threshold number
nav2/controller_server/ros__parameters/min_y_velocity_threshold number
nav2/controller_server/ros__parameters/min_theta_velocity_threshold number
nav2/controller_server/ros__parameters/failure_tolerance number
nav2/controller_server/ros__parameters/progress_checker_plugin text
nav2/controller_server/ros__parameters/goal_checker_plugins texts
nav2/controller_server/ros__parameters/controller_plugins texts
nav2/controller_server/ros__parameters/progress_checker/plugin text
nav2/controller_server/ros__parameters/progress_checker/required_movement_radius number
nav2/controller_server/ros__parameters/progress_checker/movement_time_allowance number
nav2/controller_server/ros__parameters/goal_checker/plugin text
nav2/controller_server/ros__parameters/goal_checker/xy_goal_tolerance number
nav2/controller_server/ros__parameters/goal_checker/yaw_goal_tolerance number
nav2/controller_server/ros__parameters/goal_checker/stateful bool
nav2/controller_server/ros__parameters/goal_checker/trans_stopped_velocity number
nav2/controller_server/ros__parameters/goal_checker/rot_stopped_velocity number
nav2/controller_server/ros__parameters/FollowPath/plugin text
nav2/controller_server/ros__parameters/FollowPath/heading_mode text
nav2/controller_server/ros__parameters/FollowPath/lookahead_distance number
nav2/controller_server/ros__parameters/FollowPath/projection_search_distance number
nav2/controller_server/ros__parameters/FollowPath/position_gain number
nav2/controller_server/ros__parameters/FollowPath/heading_gain number
nav2/controller_server/ros__parameters/FollowPath/coordination_linear_speed number
nav2/controller_server/ros__parameters/FollowPath/coordination_angular_speed number
nav2/controller_server/ros__parameters/FollowPath/coordination_wait_timeout number
nav2/controller_server/ros__parameters/FollowPath/coordination_direction_tolerance number
nav2/controller_server/ros__parameters/FollowPath/transform_tolerance number
nav2/controller_server/ros__parameters/FollowPath/collision_horizon number
nav2/controller_server/ros__parameters/FollowPath/collision_step number
nav2/local_costmap/local_costmap/ros__parameters/use_sim_time bool
nav2/local_costmap/local_costmap/ros__parameters/global_frame text
nav2/local_costmap/local_costmap/ros__parameters/robot_base_frame text
nav2/local_costmap/local_costmap/ros__parameters/resolution number
nav2/local_costmap/local_costmap/ros__parameters/transform_tolerance number
nav2/local_costmap/local_costmap/ros__parameters/update_frequency number
nav2/local_costmap/local_costmap/ros__parameters/publish_frequency number
nav2/local_costmap/local_costmap/ros__parameters/always_send_full_costmap bool
nav2/local_costmap/local_costmap/ros__parameters/plugins texts
nav2/local_costmap/local_costmap/ros__parameters/static_layer/plugin text
nav2/local_costmap/local_costmap/ros__parameters/static_layer/map_topic text
nav2/local_costmap/local_costmap/ros__parameters/static_layer/map_subscribe_transient_local bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/plugin text
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/observation_sources text
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/footprint_clearing_enabled bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/topic text
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/data_type text
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/marking bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/clearing bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/inf_is_valid bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/expected_update_rate number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/observation_persistence number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/obstacle_min_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/obstacle_max_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/raytrace_min_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/raytrace_max_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/min_obstacle_height number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/front/max_obstacle_height number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/topic text
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/data_type text
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/marking bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/clearing bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/inf_is_valid bool
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/expected_update_rate number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/observation_persistence number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/obstacle_min_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/obstacle_max_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/raytrace_min_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/raytrace_max_range number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/min_obstacle_height number
nav2/local_costmap/local_costmap/ros__parameters/obstacle_layer/back/max_obstacle_height number
nav2/local_costmap/local_costmap/ros__parameters/inflation_layer/plugin text
nav2/local_costmap/local_costmap/ros__parameters/inflation_layer/cost_scaling_factor number
nav2/local_costmap/local_costmap/ros__parameters/rolling_window bool
nav2/local_costmap/local_costmap/ros__parameters/width integer
nav2/local_costmap/local_costmap/ros__parameters/height integer
nav2/local_costmap/local_costmap/ros__parameters/track_unknown_space bool
nav2/global_costmap/global_costmap/ros__parameters/use_sim_time bool
nav2/global_costmap/global_costmap/ros__parameters/global_frame text
nav2/global_costmap/global_costmap/ros__parameters/robot_base_frame text
nav2/global_costmap/global_costmap/ros__parameters/resolution number
nav2/global_costmap/global_costmap/ros__parameters/transform_tolerance number
nav2/global_costmap/global_costmap/ros__parameters/update_frequency number
nav2/global_costmap/global_costmap/ros__parameters/publish_frequency number
nav2/global_costmap/global_costmap/ros__parameters/always_send_full_costmap bool
nav2/global_costmap/global_costmap/ros__parameters/plugins texts
nav2/global_costmap/global_costmap/ros__parameters/static_layer/plugin text
nav2/global_costmap/global_costmap/ros__parameters/static_layer/map_topic text
nav2/global_costmap/global_costmap/ros__parameters/static_layer/map_subscribe_transient_local bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/plugin text
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/observation_sources text
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/footprint_clearing_enabled bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/topic text
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/data_type text
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/marking bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/clearing bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/inf_is_valid bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/expected_update_rate number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/observation_persistence number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/obstacle_min_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/obstacle_max_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/raytrace_min_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/raytrace_max_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/min_obstacle_height number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/front/max_obstacle_height number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/topic text
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/data_type text
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/marking bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/clearing bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/inf_is_valid bool
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/expected_update_rate number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/observation_persistence number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/obstacle_min_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/obstacle_max_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/raytrace_min_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/raytrace_max_range number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/min_obstacle_height number
nav2/global_costmap/global_costmap/ros__parameters/obstacle_layer/back/max_obstacle_height number
nav2/global_costmap/global_costmap/ros__parameters/inflation_layer/plugin text
nav2/global_costmap/global_costmap/ros__parameters/inflation_layer/cost_scaling_factor number
nav2/global_costmap/global_costmap/ros__parameters/track_unknown_space bool
nav2/bt_navigator/ros__parameters/use_sim_time bool
nav2/bt_navigator/ros__parameters/global_frame text
nav2/bt_navigator/ros__parameters/robot_base_frame text
nav2/bt_navigator/ros__parameters/odom_topic text
nav2/bt_navigator/ros__parameters/bt_loop_duration integer
nav2/bt_navigator/ros__parameters/default_server_timeout integer
nav2/bt_navigator/ros__parameters/wait_for_service_timeout integer
nav2/bt_navigator/ros__parameters/plugin_lib_names texts
""".strip().splitlines())
_BRANCHES = {path.rsplit('/', depth)[0]
             for path in _FIELDS for depth in range(1, path.count('/') + 1)}


def normalise_nav2(parameters):
    """ROS double parameters must stay double even when YAML users enter 1."""
    result = copy.deepcopy(parameters)
    for path, kind in _FIELDS.items():
        if kind != 'number' or not path.startswith('nav2/'):
            continue
        parts = path.split('/')[1:]
        target = result
        for part in parts[:-1]:
            target = target[part]
        target[parts[-1]] = float(target[parts[-1]])
    return result


def _fail(path, message):
    raise ValueError(f"navigation.yaml:{path}: {message}")


def _leaves(value, path=""):
    if not isinstance(value, dict):
        _fail(path or "/", "必须是参数映射")
    result = {}
    for key, child in value.items():
        if not isinstance(key, str) or '/' in key:
            _fail(path or "/", "参数名称必须是不含 / 的字符串")
        name = f"{path}/{key}" if path else key
        if isinstance(child, dict):
            if name not in _BRANCHES:
                _fail(name, "未知参数映射")
            result.update(_leaves(child, name))
        else:
            result[name] = child
    return result


def validate_navigation(data, chassis):
    """Return validated settings or raise an error naming the bad parameter."""
    leaves = _leaves(data)
    missing = sorted(_FIELDS.keys() - leaves.keys())
    unknown = sorted(leaves.keys() - _FIELDS.keys())
    if unknown:
        _fail(unknown[0], "未知参数或派生参数；速度/加速度请修改 chassis.yaml，轮廓由 URDF 生成")
    if missing:
        _fail(missing[0], "缺少必要参数")
    nonnegative = {
        "footprint_padding", "inflation_clearance", "observation_persistence",
        "min_x_velocity_threshold", "min_y_velocity_threshold", "min_theta_velocity_threshold",
        "failure_tolerance", "max_iterations",
        "obstacle_min_range", "raytrace_min_range",
    }
    signed = {"min_obstacle_height", "max_obstacle_height"}
    for path, kind in _FIELDS.items():
        value = leaves[path]
        name = path.rsplit("/", 1)[-1]
        if kind == "bool":
            if type(value) is not bool:
                _fail(path, "必须为 true 或 false")
        elif kind in ("integer", "number"):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                _fail(path, "必须为有限数值")
            if kind == "integer" and not isinstance(value, int):
                _fail(path, "必须为整数")
            if name not in signed:
                zero_ok = name in nonnegative or name.endswith(".scale")
                if value < 0 or (value == 0 and not zero_ok):
                    _fail(path, "必须非负" if zero_ok else "必须大于 0")
        elif kind == "text":
            if not isinstance(value, str) or not value.strip():
                _fail(path, "必须为非空字符串")
        elif kind == "texts":
            if (not isinstance(value, list) or not value
                    or any(not isinstance(item, str) or not item.strip() for item in value)
                    or len(set(value)) != len(value)):
                _fail(path, "必须为不重复的非空字符串列表")
    if data["version"] != 1:
        _fail("version", "只支持配置版本 1")

    nav = data["nav2"]
    controller = nav["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    speed = chassis["speed"]
    for name, maximum in (('coordination_linear_speed', speed['default_linear']),
                          ('coordination_angular_speed', speed['default_angular'])):
        if not 1e-4 < follow[name] <= maximum:
            _fail('nav2/controller_server/ros__parameters/FollowPath/'+name,
                  '必须大于 0.0001 且不超过 chassis.yaml 中对应的默认速度')
    if not 0 < follow['coordination_direction_tolerance'] < math.pi/2:
        _fail('nav2/controller_server/ros__parameters/FollowPath/coordination_direction_tolerance',
              '必须大于 0 且小于 pi/2')
    wait = follow['coordination_wait_timeout']
    if not chassis['timeouts']['feedback'] < wait <= 60:
        _fail('nav2/controller_server/ros__parameters/FollowPath/coordination_wait_timeout',
              '必须大于 chassis.yaml 的反馈超时且不超过 60 秒')
    if wait >= controller['progress_checker']['movement_time_allowance']:
        _fail('nav2/controller_server/ros__parameters/FollowPath/coordination_wait_timeout',
              '必须小于 progress_checker.movement_time_allowance')
    if follow['heading_mode'] not in ('hold_initial', 'path_tangent'):
        _fail('nav2/controller_server/ros__parameters/FollowPath/heading_mode',
              '只支持 hold_initial（保持当前朝向）或 path_tangent（跟随路径并使用目标朝向）')
    # Use the gate's command timeout. Configuration must provide
    # a shorter control period and a prediction horizon beyond that period.
    period = 1.0 / controller["controller_frequency"]
    if period >= COMMAND_TIMEOUT:
        _fail("nav2/controller_server/ros__parameters/controller_frequency",
              f"控制周期必须短于 {COMMAND_TIMEOUT:g} 秒的速度指令超时")
    if follow['projection_search_distance'] < follow['lookahead_distance']:
        _fail('nav2/controller_server/ros__parameters/FollowPath/projection_search_distance', '不得小于预瞄距离')
    if follow['collision_step'] > period:
        _fail('nav2/controller_server/ros__parameters/FollowPath/collision_step', '不得大于控制周期')
    if follow['collision_horizon'] < period:
        _fail('nav2/controller_server/ros__parameters/FollowPath/collision_horizon', '不得小于控制周期')
    comparisons = {
        "min_x_velocity_threshold": speed["default_linear"],
        "min_y_velocity_threshold": speed["default_linear"],
        "min_theta_velocity_threshold": speed["default_angular"],
    }
    for name, maximum in comparisons.items():
        if controller[name] >= maximum:
            _fail("nav2/controller_server/ros__parameters/" + name, "必须小于 chassis.yaml 中对应的默认速度")
    checker = controller['goal_checker']
    if checker['xy_goal_tolerance'] >= follow['lookahead_distance']:
        _fail('nav2/controller_server/ros__parameters/goal_checker/xy_goal_tolerance', '必须小于预瞄距离')
    for name, maximum in (('trans_stopped_velocity', speed['default_linear']),
                          ('rot_stopped_velocity', speed['default_angular'])):
        if checker[name] >= maximum:
            _fail('nav2/controller_server/ros__parameters/goal_checker/' + name,
                  '必须小于 chassis.yaml 中对应的默认速度')
    # Fixed names identify the entries into which the shared builder injects
    # derived speed, timing, and geometry. A different algorithm needs code and
    # deployment support as well as a YAML edit.
    fixed = {
        "nav2/planner_server/ros__parameters/planner_plugins": ["GridBased"],
        "nav2/controller_server/ros__parameters/controller_plugins": ["FollowPath"],
        "nav2/controller_server/ros__parameters/progress_checker_plugin": "progress_checker",
        "nav2/controller_server/ros__parameters/goal_checker_plugins": ["goal_checker"],
        "nav2/controller_server/ros__parameters/FollowPath/plugin": "fairino_lookahead_controller::HolonomicLookaheadController",
        "nav2/controller_server/ros__parameters/goal_checker/plugin": "nav2_controller::StoppedGoalChecker",
        "nav2/controller_server/ros__parameters/goal_checker/stateful": False,
        "nav2/planner_server/ros__parameters/GridBased/plugin": "fairino_straight_planner::StraightLinePlanner",
        "nav2/planner_server/ros__parameters/GridBased/use_final_approach_orientation": False,
    }
    for path, expected in fixed.items():
        if leaves[path] != expected:
            _fail(path, f"当前实现要求 {expected!r}")
    for name in ("local_costmap", "global_costmap"):
        prefix = f"nav2/{name}/{name}/ros__parameters"
        costmap = nav[name][name]["ros__parameters"]
        if costmap["plugins"] != ["static_layer", "obstacle_layer", "inflation_layer"]:
            _fail(prefix + "/plugins", "当前实现要求 static_layer、obstacle_layer、inflation_layer")
        if costmap["obstacle_layer"]["observation_sources"] != "front back":
            _fail(prefix + "/obstacle_layer/observation_sources", "当前实现要求 front back")
        for scanner in ("front", "back"):
            scan = costmap["obstacle_layer"][scanner]
            for minimum, maximum in (("obstacle_min_range", "obstacle_max_range"),
                                     ("raytrace_min_range", "raytrace_max_range"),
                                     ("min_obstacle_height", "max_obstacle_height")):
                if scan[minimum] >= scan[maximum]:
                    _fail(prefix + "/obstacle_layer/" + scanner + "/" + minimum,
                          f"必须小于 {maximum}")
        if name == "local_costmap":
            if min(costmap["width"], costmap["height"]) <= costmap["resolution"]:
                _fail(prefix + "/resolution", "必须小于局部代价地图宽度和高度")
    return data
