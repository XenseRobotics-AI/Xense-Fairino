"""Calibrated vertical torso motion, using the standalone body's geometry."""
import math

from body_core import BodyModel

DEADBAND = .2
FOLD = (17, 18, 19)  # zero-based J18–J20; head, waist yaw and roll remain held.


def lift_axis(request):
    value = request.get('lift', 0.)
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1:
        raise ValueError('身体升降摇杆必须是 -1～1 的有限数值')
    return float(value)


class LiftInput:
    def __init__(self):
        self.armed = False

    def reset(self):
        self.armed = False

    def value(self, frame, active):
        right = frame.get('right', {}) if frame else {}
        if not active or not right.get('joystick_available') or right.get('axis_click'):
            self.reset()
            return 0.
        value = lift_axis({'lift': right.get('axis_y')})
        if not self.armed:
            self.armed = abs(value) <= DEADBAND
            return 0.
        return value if abs(value) > DEADBAND else 0.


class LiftPlanner:
    def __init__(self, config, speed_mm_s=15.):
        self.model = BodyModel(config)
        if not config.get('upright_confirmed'):
            raise ValueError('身体竖直参考未标定，不能启用 PICO 升降')
        if type(speed_mm_s) not in (int, float) or not math.isfinite(speed_mm_s) or speed_mm_s <= 0:
            raise ValueError('升降速度必须是有限正数 mm/s')
        self.speed = float(speed_mm_s)
        self.joint_speed = math.radians(self.model.config['default_speed_deg_s'])
        if self.joint_speed <= 0:
            raise ValueError('身体默认关节速度必须大于零')
        self.last_time = None
        self.low_mm, self.high_mm = 0., self.model.travel

    def reset(self):
        self.last_time = None

    def validated_state(self, q):
        state = self.model.state(q)
        # Feedback-seeded holds can contain tiny encoder deviations around zero.
        # Do not command J16/J17 to correct them; only ignore them for the gate.
        gate = {key: state[key] for key in ('height', 'forward', 'roll', 'yaw')}
        for key in ('forward', 'roll', 'yaw'):
            if abs(gate[key]) <= .3:
                gate[key] = 0.
        if -.001 <= gate['height'] <= 100.001:
            gate['height'] = max(0., min(100., gate['height']))
        self.model.validate(gate)
        if not 0 < q[18] < math.pi:
            raise ValueError('身体不在已标定的折叠分支')
        knee = math.degrees(q[18])
        if not self.model.config['high_knee_deg']-.3 <= knee <= self.model.low[1]+.3:
            raise ValueError('折叠关节超出已标定的高低位范围')
        return state

    def sample(self, axis, command, now):
        axis = lift_axis({'lift': axis})
        self.model.check_q(command)
        if abs(axis) <= DEADBAND:
            self.reset()
            return command[:]
        state = self.validated_state(command)
        if self.last_time is None:
            # Keep the measured horizontal offset instead of jumping onto a
            # nominal CAD line. The calibrated LOW/HIGH vertical bounds stay.
            self.model.x, _ = self.model.fold_position(math.degrees(command[18]),
                                                        math.degrees(command[19]))
            # Intersect calibrated vertical travel with calibrated knee bounds
            # at this horizontal offset, avoiding the straight-knee singularity.
            def height_at(knee_deg):
                radius2 = (self.model.l1**2+self.model.l2**2
                           +2*self.model.l1*self.model.l2*math.cos(math.radians(knee_deg)))
                return math.sqrt(max(0., radius2-self.model.x**2))-self.model.z_low
            self.low_mm = max(0., height_at(self.model.low[1]))
            self.high_mm = min(self.model.travel, height_at(self.model.config['high_knee_deg']))
            if self.high_mm <= self.low_mm:
                raise ValueError('当前水平偏移没有有效的标定升降区间')
            self.last_time = now
            return command[:]
        dt = max(0., min(.05, now-self.last_time))
        self.last_time = now
        value = math.copysign((abs(axis)-DEADBAND)/(1.-DEADBAND), axis)
        dz = value*self.speed*dt
        height_mm = state['height_mm']
        if (height_mm <= self.low_mm and dz < 0) or (height_mm >= self.high_mm and dz > 0):
            return command[:]
        target_mm = max(self.low_mm, min(self.high_mm, height_mm+dz))
        dz = target_mm-height_mm

        def candidate(scale):
            next_state = {**state, 'height': (height_mm+dz*scale)/self.model.travel*100.}
            gate = {key: next_state[key] for key in ('height', 'forward', 'roll', 'yaw')}
            for key in ('forward', 'roll', 'yaw'):
                if abs(gate[key]) <= .3:
                    gate[key] = 0.
            self.model.validate(gate)
            q = self.model.curve(next_state, command)[0]
            # Only the three folding axes change; preserve every other target.
            return [q[i] if i in FOLD else command[i] for i in range(20)]

        limit = self.joint_speed*dt
        q = candidate(1.)
        if max(abs(q[i]-command[i]) for i in FOLD) <= limit+1e-12:
            return q
        low, high = 0., 1.
        best = command[:]
        for _ in range(18):
            mid = (low+high)/2
            q = candidate(mid)
            if max(abs(q[i]-command[i]) for i in FOLD) <= limit:
                best, low = q, mid
            else:
                high = mid
        return best
