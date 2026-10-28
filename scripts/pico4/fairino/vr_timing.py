"""Shared VR timing profile and deadline scheduling; no ROS or hardware access."""
import math
from collections import deque
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Timing:
    command_hz: float = 90.
    receiver_hz: float = 200.
    feedback_hz: float = 100.
    horizon_ms: float = 0.
    interpolation_delay_ms: float = 12.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.command_hz, self.receiver_hz, self.feedback_hz)):
            raise ValueError("频率必须是有限正数")
        if not self.command_hz <= self.receiver_hz <= 500 or self.feedback_hz > 500:
            raise ValueError("接收频率须不低于下发频率；频率不得超过底层配置的 500 Hz")
        if self.horizon_ms != 0:
            raise ValueError("新版已取消关节追赶段；--horizon-ms 只能为 0，TCP 缓冲用 --interpolation-delay-ms")
        if not math.isfinite(self.interpolation_delay_ms) or not 0 <= self.interpolation_delay_ms <= 200:
            raise ValueError("TCP 插值缓冲须为 0–200 ms")

    @classmethod
    def from_args(cls, args):
        rate = getattr(args, "command_hz", 90.)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("下发频率必须是有限正数")
        return cls(rate, getattr(args, "receiver_hz", 200.),
                   getattr(args, "feedback_hz", 100.),
                   getattr(args, "horizon_ms", None) or 0.,
                   getattr(args, "interpolation_delay_ms", 12.))

    def argv(self):
        return [s for k, v in asdict(self).items() for s in ("--"+k.replace("_", "-"), str(v))]

    def payload(self):
        return asdict(self)


def add_timing_arguments(parser, *, command_default=90., receiver_default=200.,
                         command_help=None, receiver_help=None):
    parser.add_argument("--command-hz", "--interpolation-hz", dest="command_hz", type=float,
                        default=command_default,
                        help=command_help or "TCP 采样、IK 和下发频率，默认 90 Hz；两参数同义")
    parser.add_argument("--receiver-hz", type=float, default=receiver_default,
                        help=receiver_help or "机器人接收轮询频率，默认 200 Hz")
    parser.add_argument("--feedback-hz", type=float, default=100., help="机器人状态回传频率，默认 100 Hz")
    parser.add_argument("--horizon-ms", type=float, default=None,
                        help="旧参数；新版只接受 0，机器人不再添加关节插值段")
    parser.add_argument("--interpolation-delay-ms", type=float, default=12.,
                        help="IK 前 TCP 插值缓冲，默认 12 ms；0 为取最新帧，不外推")


class Deadline:
    """Absolute cadence. Skip missed slots; never replay an overdue backlog."""
    def __init__(self, hz, now):
        self.period = 1./hz
        self.next = now
        self.skipped = 0

    def advance(self, now):
        self.next += self.period
        if self.next < now:
            missed = math.floor((now-self.next)/self.period)+1
            self.next += missed*self.period
            self.skipped += missed
        return max(0., self.next-now)


class RateMeter:
    def __init__(self, window=2.):
        self.times = deque()
        self.window = window
        self.total = 0

    def mark(self, now):
        self.times.append(now)
        self.total += 1

    def hz(self, now):
        while self.times and self.times[0] < now-self.window:
            self.times.popleft()
        if len(self.times) < 2:
            return 0.
        return (len(self.times)-1)/max(1e-9, self.times[-1]-self.times[0])
