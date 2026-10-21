"""Hardware-free validation and identity selection. Position 0=closed, 1=open."""
import math

SIDES = ('left', 'right')
LEASE = .25
DEFAULT_TORQUE = 1.1  # Nm, bounded again against the connected motor's spec.
DEFAULT_SPEED = .25  # shaft rad/s
TELEOP_SPEED = .75  # shaft rad/s; PICO default, independent of slow manual debugging.
TARGET_PERIOD = .01  # 100 Hz latest-target transport; the SDK motor stream is 100 Hz.


def diagnostic_snapshot(fault, audit, firmware='未知'):
    """Cached reads only; this can be rendered while the SDK owns the stream."""
    fields = ('version', 'report_flags', 'source_mask', 'protocol_mode',
              'motor_fault_code', 'motor_latched_fault_code',
              'firmware_fault_code', 'firmware_latched_fault_code')
    values = {key: int(getattr(fault, key)) for key in fields}
    envelope = {'ok': bool(audit.ok), 'detail': audit.detail}
    for name in ('stored', 'effective', 'recommended'):
        record = getattr(audit, name, None)
        envelope[name] = (dict(cont_nm=float(record.cont_torque_nm),
                               peak_nm=float(record.peak_torque_nm))
                          if record is not None else None)
    return {'firmware': firmware, 'fault': values, 'envelope': envelope,
            'issues': preflight_issues(values, envelope, firmware)}


def preflight_issues(fault, envelope, firmware='未知'):
    """Validate fault reports, including the reviewed 1.2.16 zero-fault layout."""
    issues = []
    flags = fault['report_flags']
    missing = []
    if not flags & 0x01:
        missing.append('报告 Valid')
    if not flags & 0x04:
        missing.append('电机 MotorValid')
    # In the shipped 1.2.16 image (CRC32 0x4A8CD360), 0x08021146 writes
    # both firmware fault words even when zero; 0x080211E4 only sets FwValid
    # when that word is nonzero. The 0x52 handler also sets it for control
    # errors/deadline history. See README for the checked image/addresses.
    # Keep the strict fallback for unreviewed firmware/layouts or MCU sources.
    known_zero = (firmware == 'FirmwareVersion(1.2.16.0)'
                  and fault.get('version') == 1
                  and fault.get('protocol_mode') == 2
                  and fault.get('source_mask') in (0, 1)
                  and fault['firmware_fault_code'] == 0
                  and fault['firmware_latched_fault_code'] == 0)
    if not flags & 0x08 and not known_zero:
        missing.append('固件 FwValid')
    if missing:
        issues.append(f'故障诊断有效标记不完整（flags=0x{flags:02X}，缺少'+ '、'.join(missing)+'）')
    if fault.get('version') != 1:
        issues.append('故障报告协议版本未确认或不受支持')
    if flags & 0x20:
        issues.append('故障读取超时，拒绝使用该次诊断')
    if flags & 0x40:
        issues.append('故障报告为私有协议部分数据，无法完成使能检查')
    # Keep the existing refusal of nonzero raw motor fields as well as missing
    # validity. Explain the value without claiming it is trustworthy when invalid.
    for field, title in (('motor_fault_code', '电机当前故障'),
                         ('motor_latched_fault_code', '电机历史故障')):
        value = fault[field]
        if value:
            name = '（SDK 故障表：欠压）' if value == 0x04 and flags & 0x04 else ''
            trust = '' if flags & 0x04 else '（有效性未确认）'
            issues.append(f'{title}=0x{value:08X}{name}{trust}')
    for field, title in (('firmware_fault_code', '固件当前故障'),
                         ('firmware_latched_fault_code', '固件历史故障')):
        value = fault[field]
        if value and not (field == 'firmware_latched_fault_code' and value == 0x0303):
            trust = '' if flags & 0x08 else '（有效性未确认）'
            issues.append(f'{title}=0x{value:08X}{trust}')
    if not envelope['ok'] or envelope['effective'] is None:
        stored, recommended = envelope['stored'], envelope['recommended']
        if stored is not None and recommended is not None:
            issues.append('力矩包络校验未通过：已存持续/峰值 '
                f'{stored["cont_nm"]:.3f}/{stored["peak_nm"]:.3f} Nm，SDK 建议 '
                f'{recommended["cont_nm"]:.3f}/{recommended["peak_nm"]:.3f} Nm')
        else:
            issues.append('力矩包络校验未通过：'+envelope['detail'])
    return issues


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
    def __init__(self, max_openings=None):
        values = max_openings if max_openings is not None else dict.fromkeys(SIDES, 1.)
        if not isinstance(values, dict) or set(values) != set(SIDES):
            raise ValueError('最大开度必须包含左右夹爪')
        self.max_openings = {side: opening(values[side]) for side in SIDES}
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
                result[side] = self.max_openings[side]*(1.-trigger)
        return result
