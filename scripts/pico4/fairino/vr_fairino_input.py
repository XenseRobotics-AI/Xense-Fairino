"""Express clutch anchors, interpolation and wrist rebasing in a fixed virtual base."""
import numpy as np

import paths  # Locate the shared robot-side TCP modules before importing CartesianInput.
from vr_cartesian import CartesianInput


class FairinoInput(CartesianInput):
    def __init__(self, clutches, backend, delay_ms=12., *, automatic=False):
        super().__init__(clutches, delay_ms, automatic=automatic)
        self.backend = backend

    def reset(self):
        super().reset()
        self.backend.reset()

    def goals(self, frame, hw, kin, hands, sensitivity, freeze_buttons, now):
        q = np.asarray(hw['position' if self.automatic else 'command_position'], dtype=float)
        backend = self.backend

        class FixedKinematics:
            def forward_tcp(self, arm, side):
                tcp, angle = kin.forward_tcp(arm, side)
                return backend.teleop_tcp(side, q, tcp), angle

        # The client skips request() when both Grips are released. A short
        # release/regrip can also occur within a single batch of input frames.
        samples = frame.get('samples', [frame])
        released = not self.automatic and any(all(sample[hand]['grip'] <= .3 for hand in hands.values()) for sample in samples)
        goals = super().goals(frame, hw, FixedKinematics(), hands, sensitivity, freeze_buttons, now)
        if released or not goals:
            self.backend.reset()
        return goals

    def accepted_plan(self, packet, frame=None, hands=None):
        entries = {s: dict(entry, tcp=entry['teleop_tcp']) for s, entry in packet['arms'].items()}
        super().accepted_plan(dict(packet, arms=entries), frame, hands)
