"""Optional wrist envelope, independent of ROS, GTK, IK and mesh libraries.

Only forearm2_Link vs waist2_Link is covered (J6/J7 and J13/J14).
The sampled visual meshes are not a certified collision model. See
docs/腕部保护.md for provenance, margins and the runtime switching contract.
All public motion inputs are radians; the embedded table is degrees.
"""
from bisect import bisect_left
import hashlib
import json
import math

VERSION = "wrist-envelope-20261020-v1"
MODEL_SHA256 = "d64b19f6fc825ecdd2c97dbeb26514f844cba2ce28fa010864c0eed11e190cbe"
# Minimum last nonintersection angle over both arms and both signs.
# A cell uses its lower cap, never an interpolated larger free region.
BOUNDARY = (
    (0, 85), (1, 85), (2, 85), (4, 80.5), (5, 77.6), (6, 73.7),
    (8, 66.3), (10, 61.4), (12, 58), (14, 55.5), (16, 53.5),
    (18, 51.9), (20, 50.6), (22, 49.5), (24, 48.5), (26, 47.6),
    (28, 46.8), (30, 46.1), (32, 45.5), (34, 44.9), (36, 44.3),
    (38, 43.7), (40, 43.2), (42, 42.7), (44, 42.1), (46, 41.6),
    (48, 41.2), (50, 40.7), (52, 40.2), (54, 39.7), (56, 39.2),
    (58, 38.6), (60, 38.1), (62, 37.6), (64, 37.1), (66, 36.5),
    (68, 35.9), (70, 35.3),
)
MARGIN_DEG = 2.0  # Erode BOTH wrist axes; this is not a measured braking distance.
SLOW_BAND_DEG = 5.0
PREDICTION_SECONDS = .20  # Additional velocity envelope for PICO direct setpoints.
CONFIG_ID = hashlib.sha256(json.dumps(
    [VERSION, MODEL_SHA256, BOUNDARY, MARGIN_DEG, SLOW_BAND_DEG, PREDICTION_SECONDS]
).encode()).hexdigest()[:16]
_ANGLES = tuple(row[0] for row in BOUNDARY)
_PAIRS = ((5, 6, "左腕 J6/J7"), (12, 13, "右腕 J13/J14"))


class WristBlocked(ValueError):
    """An ordinary motion refusal; callers hold enabled instead of exiting."""


def policy(enabled=True, epoch=1):
    if type(enabled) is not bool or type(epoch) is not int or epoch < 1:
        raise ValueError("腕部模块配置无效")
    return dict(version=VERSION, config_id=CONFIG_ID, enabled=enabled, epoch=epoch)


def validate_policy(value):
    if not isinstance(value, dict) or set(value) != set(policy()):
        raise ValueError("腕部模块配置缺失")
    expected = policy(value["enabled"], value["epoch"])
    if value != expected:
        raise ValueError("腕部模块版本 / 边界不一致")
    return expected


def _vector(value, count):
    if not isinstance(value, (tuple, list)) or len(value) != count:
        raise WristBlocked("腕部保护收到无效关节向量")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise WristBlocked("腕部保护收到非有限角度 / 速度")
    return value


def cap(q6):
    """Conservative, symmetric step envelope. q6 is an absolute degree angle."""
    if not math.isfinite(q6) or q6 < 0 or q6 > 70:
        return -math.inf
    return BOUNDARY[min(bisect_left(_ANGLES, q6), len(BOUNDARY)-1)][1]


class WristGuard:
    def __init__(self, enabled=True):
        if type(enabled) is not bool:
            raise ValueError("腕部模块启用状态必须为布尔值")
        self.enabled = enabled

    @staticmethod
    def clearance(pair, margin=MARGIN_DEG, reserve=(0., 0.)):
        x, y = (abs(math.degrees(v)) for v in pair)
        x += margin + reserve[0]
        return min(70-x, cap(x)-margin-reserve[1]-y)

    @staticmethod
    def _box_clear(points, reserve=(0., 0.)):
        x = max(abs(p[0]) for p in points)
        y = max(abs(p[1]) for p in points)
        return WristGuard.clearance((x, y), reserve=reserve) >= -1e-9

    def _segment(self, a, b, reserve=(0., 0.), depth=0):
        if self._box_clear((a, b), reserve):
            return True
        if (self.clearance(a, reserve=reserve) < -1e-9
                or self.clearance(b, reserve=reserve) < -1e-9 or depth >= 14):
            return False
        mid = tuple((u+v)*.5 for u, v in zip(a, b))
        return (self._segment(a, mid, reserve, depth+1)
                and self._segment(mid, b, reserve, depth+1))

    def _pair_motion(self, a, b, reserve=(0., 0.), allow_retreat=True):
        if self._segment(a, b, reserve):
            return True
        # Manual retreat from the margin band: each absolute wrist angle must
        # decrease or hold, without crossing zero. Never auto-generate an escape.
        if (allow_retreat and self.clearance(a, margin=0) >= 0 and self.clearance(b, margin=0) >= 0
                and self.clearance(a, reserve=reserve) < 0):
            return all(abs(v) <= abs(u)+1e-10 and u*v >= -1e-12 for u, v in zip(a, b))
        return False

    def arm_segment(self, start, target, velocity=None, label="腕部"):
        start, target = _vector(start, 7), _vector(target, 7)
        if velocity is not None:
            _vector(velocity, 7)
        if not self.enabled:
            return
        reserve = ((0., 0.) if velocity is None else
                   tuple(abs(math.degrees(v))*PREDICTION_SECONDS for v in velocity[5:7]))
        inward = velocity is None or all(p*v <= 1e-10 for p, v in zip(start[5:7], velocity[5:7]))
        if not self._pair_motion(start[5:7], target[5:7], reserve, inward):
            raise WristBlocked(f"{label}联合限位：目标或经过路径接近前臂外壳；请向内收腕")

    def segment(self, start, target):
        _vector(start, 20); _vector(target, 20)
        for offset, label in ((0, "左腕 J6/J7"), (7, "右腕 J13/J14")):
            self.arm_segment(start[offset:offset+7], target[offset:offset+7], label=label)

    def approach_scale(self, start, target):
        """Slow only motion that approaches the margin; inward retreat stays free."""
        if not self.enabled:
            return 1.
        before, after = self.clearance(start[5:7]), self.clearance(target[5:7])
        if after >= before-1e-8:
            return 1.
        return max(0., min(1., before/SLOW_BAND_DEG))

    def _bezier(self, control, depth=0):
        # The convex hull contains the entire cubic, including velocity
        # reversal extrema. Subdivision tightens an over-conservative hull.
        if self._box_clear(control):
            return True
        if depth >= 12:
            return False
        if self.clearance(control[0]) < -1e-9 or self.clearance(control[-1]) < -1e-9:
            # Monotonic inward retreat is also valid for a cubic if all its
            # control coordinates move monotonically toward zero.
            return (self.clearance(control[0], margin=0) >= 0
                    and self.clearance(control[-1], margin=0) >= 0
                    and all(all(abs(b[i]) <= abs(a[i])+1e-10 and a[i]*b[i] >= -1e-12
                                for a, b in zip(control, control[1:])) for i in (0, 1)))
        a, b, c, d = control
        mid = lambda u, v: tuple((x+y)*.5 for x, y in zip(u, v))
        ab, bc, cd = mid(a, b), mid(b, c), mid(c, d)
        abc, bcd = mid(ab, bc), mid(bc, cd)
        center = mid(abc, bcd)
        return (self._bezier((a, ab, abc, center), depth+1)
                and self._bezier((center, bcd, cd, d), depth+1))

    def trajectory(self, points):
        """Check the FULL cubic Hermite trajectory, including its braking tail."""
        if not isinstance(points, (tuple, list)) or len(points) < 2:
            raise WristBlocked("腕部保护需要完整的轨迹与停止尾段")
        for t, q, v in points:
            _vector(q, 20); _vector(v, 20)
            if not math.isfinite(t):
                raise WristBlocked("腕部保护收到无效轨迹时间")
        if not self.enabled:
            return
        for (t0, p0, v0), (t1, p1, v1) in zip(points, points[1:]):
            dt = t1-t0
            if dt <= 0:
                raise WristBlocked("腕部保护收到倒退轨迹时间")
            for i, j, label in _PAIRS:
                control = (tuple(p0[k] for k in (i, j)),
                           tuple(p0[k]+v0[k]*dt/3 for k in (i, j)),
                           tuple(p1[k]-v1[k]*dt/3 for k in (i, j)),
                           tuple(p1[k] for k in (i, j)))
                if not self._bezier(control):
                    raise WristBlocked(f"{label}联合限位：轨迹 / 减速尾段越界；保持并换方向")


class GuardSession:
    """Receiver-side activation fence; only exchange modules while idle."""
    def __init__(self):
        self.active = None
        self.guard = WristGuard()  # Missing config never means disabled.
        self.message = "等待腕部模块双端确认"
        self.error = ""
        self.floor = 0

    def status(self):
        return dict(policy=self.active, message=self.message, error=self.error,
                    version=VERSION, config_id=CONFIG_ID, margin_deg=MARGIN_DEG)

    def filter(self, packet, hw, idle):
        request = packet.get("request")
        try:
            if request is not None and (not isinstance(request, dict) or type(request.get("id")) is not int):
                raise ValueError("腕部模块收到无效运动请求")
            wanted = validate_policy(packet.get("wrist_guard"))
            if self.active and wanted["epoch"] < self.active["epoch"]:
                raise ValueError("拒绝旧腕部模块配置")
            if self.active and wanted["epoch"] == self.active["epoch"] and wanted != self.active:
                raise ValueError("腕部配置变更必须递增版本序号")
        except ValueError as exc:
            self.error = str(exc)
            return None
        self.error = ""
        if wanted != self.active:
            self.floor = max(self.floor, (request or {}).get("id", 0))
            self.message = "切换腕部模块：等待暂停并静止"
            if (request is None and idle and hw is not None
                    and max(abs(v) for v in hw["velocity"]) < .005
                    and max(abs(v) for v in hw.get("command_velocity", hw["velocity"])) < .005):
                self.active = wanted
                self.guard = WristGuard(wanted["enabled"])
                self.message = ("腕部联合限位已启用" if wanted["enabled"] else "腕部联合限位已停用")
            return None
        if request is not None:
            if (not isinstance(request, dict) or request.get("_wrist_guard") != wanted
                    or type(request.get("id")) is not int or request["id"] <= self.floor):
                self.error = "已丢弃切换前的旧运动目标"
                return None
        return request


class GuardClient:
    """Client adapter shared by standalone apps and embedded panels."""
    def __init__(self, enabled=True):
        self.wanted = policy(enabled)
        self.floor = 0
        self.released = False
        self.last_id = 0

    def switch(self, enabled):
        self.wanted = policy(enabled, self.wanted["epoch"]+1)
        self.released = False
        self.floor = self.last_id

    def ready(self, status):
        return (isinstance(status, dict) and status.get("policy") == self.wanted and not status.get("error"))

    def filter(self, request, status):
        if request is not None:
            self.last_id = max(self.last_id, request.get("id", 0))
        if not self.ready(status) or not self.released:
            if request is not None:
                self.floor = max(self.floor, request.get("id", 0))
            elif self.ready(status):
                self.released = True
            return None
        if request is None:
            return None
        if request.get("id", 0) <= self.floor:
            return None
        self.floor = max(self.floor, request["id"]-1)
        return {**request, "_wrist_guard": self.wanted.copy()}

    def guard(self):
        return WristGuard(self.wanted["enabled"])
