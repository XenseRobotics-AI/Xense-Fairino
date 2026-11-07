"""Build Nav2 parameters from editable settings and shared chassis limits."""
import json
import math

from fairino_slam.configuration import load_chassis, load_navigation
from fairino_slam.navigation.settings import normalise_nav2


# Stay within the live footprint observer's 64-vertex transport limit.
_GLOBAL_CIRCLE_VERTICES = 64


def _global_footprint(radius, heading_mode):
    """Use the circle for held heading; retain the square for path heading."""
    if heading_mode != 'hold_initial':
        # Preserve the original conservative square for other strategies.
        return ([[-radius, -radius], [radius, -radius],
                 [radius, radius], [-radius, radius]], radius)
    # Smac2D uses the inflation layer's inscribed radius. Circumscribe the
    # padded chassis circle so polygon edges never reduce that protection.
    # Nav2 parses footprint coordinates as floats; keep rounding outward.
    circle_radius = radius + max(1e-6, radius * 1e-6)
    vertex_radius = circle_radius / math.cos(math.pi / _GLOBAL_CIRCLE_VERTICES)
    circle = [
        [vertex_radius * math.cos(2 * math.pi * i / _GLOBAL_CIRCLE_VERTICES),
         vertex_radius * math.sin(2 * math.pi * i / _GLOBAL_CIRCLE_VERTICES)]
        for i in range(_GLOBAL_CIRCLE_VERTICES)]
    return circle, circle_radius


def build_profile(points, linear, angular):
    chassis = load_chassis()
    speed = chassis['speed']
    for value, maximum in ((linear, speed['max_linear']), (angular, speed['max_angular'])):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0 < value <= maximum):
            raise ValueError('Invalid chassis speed limits')
    settings = load_navigation()
    profile = normalise_nav2(settings['nav2'])
    controller = profile['controller_server']['ros__parameters']
    for name, maximum in (('coordination_linear_speed', linear),
                          ('coordination_angular_speed', angular)):
        if controller['FollowPath'][name] > maximum:
            raise ValueError('navigation.yaml:FollowPath:'+name+
                             ': coordination speed must not exceed the requested navigation speed')
    geometry = settings['geometry']
    padding = geometry['footprint_padding']
    if (len(points) < 3 or any(len(point) != 2 for point in points)
            or any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for point in points for value in point)):
        raise ValueError('Invalid chassis footprint')
    radius = max(math.hypot(abs(x) + padding, abs(y) + padding) for x, y in points)
    global_footprint, inscribed_radius = _global_footprint(
        radius, controller['FollowPath']['heading_mode'])
    inflation = max(geometry['minimum_inflation_radius'],
                    radius + geometry['inflation_clearance'])
    if inflation <= inscribed_radius:
        raise ValueError('navigation.yaml:geometry: inflation must exceed the padded chassis radius')
    for name, global_map in (('local_costmap', False), ('global_costmap', True)):
        costmap = profile[name][name]['ros__parameters']
        costmap['footprint'] = json.dumps(global_footprint if global_map else points)
        costmap['footprint_padding'] = 0. if global_map else padding
        costmap['inflation_layer']['inflation_radius'] = inflation
    linear_acceleration = chassis['limits']['linear']['max_acceleration']
    angular_acceleration = chassis['limits']['angular']['max_acceleration']
    goal_checker = controller['goal_checker']
    controller['FollowPath'].update(
        max_linear_speed=float(linear), max_angular_speed=float(angular),
        linear_acceleration=float(linear_acceleration), angular_acceleration=float(angular_acceleration),
        xy_goal_tolerance=goal_checker['xy_goal_tolerance'],
        yaw_goal_tolerance=goal_checker['yaw_goal_tolerance'],
        control_period=1. / controller['controller_frequency'],
        chassis_feedback_timeout=float(chassis['timeouts']['feedback']))
    return profile
