"""Hardware-free validation and identity selection. Position 0=closed, 1=open."""
import math

SIDES = ('left', 'right')
LEASE = .25
DEFAULT_TORQUE = 1.1  # Nm, bounded again against the connected motor's spec.
DEFAULT_SPEED = .25  # shaft rad/s


def number(value, low, high, label):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label}必须在 {low:g}～{high:g} 范围内且为有限数值')
    return float(value)


def opening(value):
    return number(value, 0., 1., '夹爪开度')


def select_devices(endpoints, assignments=None):
    """SDK sides unless a firmware SN is explicitly assigned; never pick by USB order."""
    assignments = assignments or {}
    reserved = {sn for sn in assignments.values() if sn}
    selected = {}
    for side in SIDES:
        sn = assignments.get(side, '')
        matches = [ep for ep in endpoints if ep['role'] == 'Follower'
                   and not ep.get('in_use') and ep['sn']
                   and (ep['sn'] == sn if sn else ep['side'].lower() == side and ep['sn'] not in reserved)]
        if len(matches) > 1:
            raise ValueError(f'{side} 匹配多个夹爪，请按固件 SN 指定')
        if sn and not matches:
            raise ValueError(f'{side} 未找到固件 SN {sn}，拒绝改用其他设备')
        if matches:
            selected[side] = matches[0]
    if len({ep['port'] for ep in selected.values()}) != len(selected):
        raise ValueError('左右夹爪不能指向同一个设备')
    return selected


def fresh_request(request, now):
    if not isinstance(request, dict) or type(request.get('id')) is not int or request['id'] <= 0:
        raise ValueError('夹爪命令序号无效')
    created = request.get('created')
    if type(created) not in (int, float) or not math.isfinite(created) or not 0 <= now-created <= LEASE:
        raise ValueError('夹爪命令超过 0.25 秒未更新')


def healthy_sample(position, velocity, torque, temperature, status):
    if not all(type(v) in (int, float) and math.isfinite(v)
               for v in (position, velocity, torque, temperature)):
        raise RuntimeError('夹爪反馈包含无效数值')
    if not 0 <= position <= 1:
        raise RuntimeError('夹爪位置超出标定范围')
    if int(status) & ~1:
        raise RuntimeError(f'夹爪电机状态异常：0x{int(status):04X}')
    if not 0 <= temperature < 55:
        raise RuntimeError('夹爪电机温度异常或达到 55°C')


class TriggerMapping:
    """An explicit press starts tracking; neutral input on startup never opens a jaw."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.armed = set()
        self.engaged = set()

    def update(self, frame, hands, connected):
        result = {}
        for side in connected:
            controller = frame[hands[side]]
            if not controller.get('trigger_available', False):
                raise ValueError(f'{hands[side]} 手柄缺少 Trigger 数据，夹爪保持')
            trigger = opening(controller['trigger'])
            if side not in self.armed:
                if trigger <= .02:
                    self.armed.add(side)
                continue
            if trigger > .02:
                self.engaged.add(side)
            if side in self.engaged:
                result[side] = 1.-trigger
        return result
