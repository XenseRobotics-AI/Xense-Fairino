"""Read the single configuration directory, with explicit validation errors."""
import math

import yaml

from fairino_slam.paths import CONFIG_ROOT


class ConfigurationError(ValueError):
    """A configuration cannot be used; the message names the file and field."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ConfigurationError('配置字段名必须为字符串')
        if key in result:
            raise ConfigurationError(f'重复配置字段：{key}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def read_yaml(name):
    path = CONFIG_ROOT / name
    try:
        data = yaml.load(path.read_text(encoding='utf-8'), Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError, ConfigurationError) as exc:
        raise ConfigurationError(f'{path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ConfigurationError(f'{path}: 顶层必须是参数映射')
    return data


def _keys(value, expected, field):
    if not isinstance(value, dict):
        raise ConfigurationError(f'{field}: 必须为参数映射')
    missing, extra = set(expected) - value.keys(), value.keys() - set(expected)
    if missing or extra:
        raise ConfigurationError(f'{field}: 缺少字段 {sorted(missing)}；未知字段 {sorted(extra)}')


def _number(value, field, *, zero=False, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (value < 0 if zero else value <= 0)
            or (integer and not isinstance(value, int))):
        raise ConfigurationError(f'{field}: 必须为有限的{"非负" if zero else "正"}{"整数" if integer else "数值"}')
    return value


def validate_chassis(data):
    _keys(data, ('version', 'speed', 'timeouts', 'limits'), 'chassis.yaml')
    if type(data['version']) is not int or data['version'] != 1:
        raise ConfigurationError('chassis.yaml.version: 只支持版本 1')
    sections = {
        'speed': ('default_linear', 'default_angular', 'min_speed', 'max_linear', 'max_angular'),
        'timeouts': ('feedback', 'link'),
        'limits.linear': ('max_acceleration', 'max_jerk'),
        'limits.angular': ('max_acceleration', 'max_jerk'),
        'limits.steering': ('max_velocity', 'max_acceleration'),
        'limits.drive': ('max_velocity', 'max_acceleration', 'max_jerk', 'feedback_max_velocity'),
    }
    _keys(data['limits'], ('linear', 'angular', 'steering', 'drive'), 'chassis.yaml.limits')
    for field, keys in sections.items():
        value = data
        for part in field.split('.'):
            value = value[part]
        _keys(value, keys, 'chassis.yaml.' + field)
        for key in keys:
            _number(value[key], f'chassis.yaml.{field}.{key}')
    speed = data['speed']
    for axis in ('linear', 'angular'):
        if not speed['min_speed'] <= speed['default_' + axis] <= speed['max_' + axis]:
            raise ConfigurationError(f'chassis.yaml.speed: min_speed ≤ default_{axis} ≤ max_{axis} 必须成立')
    drive = data['limits']['drive']
    if drive['max_velocity'] > drive['feedback_max_velocity']:
        raise ConfigurationError('chassis.yaml.limits.drive: max_velocity 不得超过 feedback_max_velocity')
    return data


def load_chassis():
    return validate_chassis(read_yaml('chassis.yaml'))


def load_navigation():
    # Local import keeps the configuration reader independent of profile generation.
    from fairino_slam.navigation.settings import validate_navigation
    try:
        return validate_navigation(read_yaml('navigation.yaml'), load_chassis())
    except ValueError as exc:
        raise ConfigurationError(f'{CONFIG_ROOT / "navigation.yaml"}: {exc}') from exc


def load_ros_profile(name):
    """Validate essential SLAM/AMCL ranges while retaining native ROS parameters."""
    nodes = {'mapping.yaml': 'slam_toolbox', 'localization.yaml': 'amcl'}
    if name not in nodes:
        raise ConfigurationError(f'不支持的 ROS 配置：{name}')
    data = read_yaml(name)
    node = nodes[name]
    if not isinstance(data.get(node), dict) or not isinstance(data[node].get('ros__parameters'), dict):
        raise ConfigurationError(f'{name}: 缺少 {node}.ros__parameters')
    params = data[node]['ros__parameters']
    if name == 'mapping.yaml':
        fields = ('resolution', 'map_update_interval', 'minimum_time_interval',
                  'minimum_travel_distance', 'minimum_travel_heading',
                  'min_laser_range', 'max_laser_range', 'transform_timeout',
                  'tf_buffer_duration', 'transform_publish_period')
        integers = ('throttle_scans', 'scan_queue_size', 'scan_buffer_size')
        pairs = (('min_laser_range', 'max_laser_range'),)
    else:
        fields = ('update_min_d', 'update_min_a', 'transform_tolerance',
                  'laser_min_range', 'laser_max_range', 'laser_likelihood_max_dist', 'sigma_hit')
        integers = ('min_particles', 'max_particles', 'max_beams', 'resample_interval')
        pairs = (('laser_min_range', 'laser_max_range'), ('min_particles', 'max_particles'))
    for field in (*fields, *integers):
        if field not in params:
            raise ConfigurationError(f'{name}: 缺少 {node}.ros__parameters.{field}')
        _number(params[field], f'{name}.{node}.{field}', integer=field in integers,
                zero=field in ('min_laser_range', 'laser_min_range', 'minimum_travel_distance',
                               'minimum_travel_heading', 'update_min_d', 'update_min_a'))
    for lower, upper in pairs:
        if params[lower] > params[upper]:
            raise ConfigurationError(f'{name}: {lower} 不得超过 {upper}')
    return data
