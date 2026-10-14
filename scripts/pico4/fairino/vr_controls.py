"""Physical joystick edges, independent of left/right arm routing."""

LEVELS = (.25, .5, 1.)


def sensitivity_step(value, direction):
    if direction > 0:
        return next((v for v in LEVELS if v > value+1e-6), max(value, LEVELS[-1]))
    return next((v for v in reversed(LEVELS) if v < value-1e-6), min(value, LEVELS[0]))


class JoystickControls:
    def __init__(self):
        self.reset()

    def reset(self):
        self.armed = False
        self.deflected = False
        self.left_click = self.right_click = False

    def update(self, frame):
        left, right = frame["left"], frame["right"]
        y = left.get("axis_y", 0.)
        lc, rc = left.get("axis_click", False), right.get("axis_click", False)
        result = {"sensitivity": 0, "home": False, "pause": False}
        if not self.armed:
            # Reconnect or startup never turns an already held control into a new press.
            self.armed = abs(y) <= .3 and not lc and not rc
        else:
            if abs(y) <= .3:
                self.deflected = False
            elif abs(y) >= .65 and not self.deflected:
                result["sensitivity"] = 1 if y > 0 else -1
                self.deflected = True
            result["home"] = rc and not self.right_click
            result["pause"] = lc and not self.left_click
        self.left_click, self.right_click = lc, rc
        return result
