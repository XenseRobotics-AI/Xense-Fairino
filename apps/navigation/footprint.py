"""Selectable 2D footprint providers; coordinates are in the runtime base_link."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping, Protocol


class FootprintMode(str, Enum):
    CHASSIS = 'chassis'
    UPPER_BODY_PROJECTION = 'upper_body_projection'


@dataclass(frozen=True)
class FootprintRequest:
    # SI units; joints are measured feedback, never commanded target positions.
    stamp: float
    joint_positions: Mapping[str, float]
    joint_stamp: float | None = None


@dataclass(frozen=True)
class Footprint:
    # Counterclockwise enclosing polygon, metres, relative to the chassis centre.
    points: tuple[tuple[float, float], ...]
    stamp: float
    frame_id: str = 'base_link'

    def __post_init__(self):
        if self.frame_id != 'base_link' or len(self.points) < 3:
            raise ValueError('Footprint requires >=3 vertices in base_link')
        if not math.isfinite(self.stamp) or not all(
                len(p) == 2 and all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError('Footprint contains invalid coordinates/timestamp')
        area = sum(a[0]*b[1]-b[0]*a[1]
                   for a, b in zip(self.points, self.points[1:]+self.points[:1]))
        if area <= 0:
            raise ValueError('Footprint must be nondegenerate and counterclockwise')


class FootprintProvider(Protocol):
    def compute(self, request: FootprintRequest) -> Footprint:
        """Return the enclosing polygon; reject missing/stale pose feedback."""
        ...


class FootprintSelector:
    """Explicit provider selection; never silently fall back to a smaller shape."""
    def __init__(self, providers: Mapping[FootprintMode, FootprintProvider]):
        self.providers = dict(providers)

    def compute(self, mode: FootprintMode | str, request: FootprintRequest) -> Footprint:
        mode = FootprintMode(mode)
        if mode not in self.providers:
            raise NotImplementedError(f'Footprint provider not implemented: {mode.value}')
        return self.providers[mode].compute(request)


def convex_hull(points):
    points = sorted(set(points))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-12:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-12:
            upper.pop()
        upper.append(p)
    return tuple(lower[:-1]+upper[:-1])


class ChassisFootprint:
    """Project the runtime URDF base_link collision boxes onto its XY plane."""
    def __init__(self, urdf):
        import itertools
        import xml.etree.ElementTree as ET
        root = ET.parse(urdf).getroot()
        link = root.find("./link[@name='base_link']")
        if link is None:
            raise ValueError('URDF has no base_link')
        points = []
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                raise ValueError('Chassis collision must be a box; unsupported geometry is never ignored')
            size = [float(v) for v in box.attrib['size'].split()]
            origin = collision.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            if any(len(v) != 3 for v in (size, xyz, rpy)) or not all(math.isfinite(v) for v in size+xyz+rpy) or min(size) <= 0:
                raise ValueError('Invalid URDF collision size/origin')
            r, p, y = rpy
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            # First two rows of Rz(yaw) Ry(pitch) Rx(roll).
            rows = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                    (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr))
            for signs in itertools.product((-1, 1), repeat=3):
                v = [a*b/2 for a, b in zip(size, signs)]
                points.append(tuple(round(xyz[i]+sum(a*b for a, b in zip(row, v)), 12)
                                    for i, row in enumerate(rows)))
        self.points = convex_hull(points)
        Footprint(self.points, 0.)

    def compute(self, request: FootprintRequest) -> Footprint:
        return Footprint(self.points, request.stamp)


def selector(urdf):
    return FootprintSelector({FootprintMode.CHASSIS: ChassisFootprint(urdf)})
