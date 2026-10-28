"""Offline gesture and trajectory tests for high/lower torso presets."""
import math
import unittest

import paths  # noqa: F401 -- adds the shared pure body mathematics directory.
from body_core import bezier, load_config
from vr_lift import DEADBAND, FOLD
from vr_lift_presets import (
    HIGH_PERCENT, HOLD_SECONDS, MAX_OBSERVATION_GAP_S, LOW_PERCENT,
    SPEED_DEG_S, TARGET_TOLERANCE_MM, PresetLiftPlanner,
)


class PresetLiftTests(unittest.TestCase):
    def setUp(self):
        self.model = load_config()
        self.lift = PresetLiftPlanner(self.model.config)
        self.q = self.pose(70.,forward=8.,yaw=12.)

    def pose(self,height,forward=0.,roll=0.,yaw=0.):
        return self.model.pose(dict(height=height,forward=forward,roll=roll,yaw=yaw),
                               [.01*(j+1) for j in range(20)])

    def trigger(self,direction,q=None,start=10.,planner=None):
        q = self.q if q is None else q
        planner = self.lift if planner is None else planner
        planner.observe(0.,q,start)
        # Candidate starts at start+.1, then ten fresh samples complete 1 s.
        for i in range(11):
            planner.observe(direction,q,start+.1+i*.1)
        return start+1.1

    def finish(self,q,start):
        duration=self.lift.snapshot(start)['duration_s']
        return self.lift.sample(q,start+duration+.001)

    def test_constants_and_initial_neutral_requirement(self):
        self.assertEqual((HOLD_SECONDS,SPEED_DEG_S,HIGH_PERCENT),(1.,30.,100.))
        self.assertAlmostEqual(LOW_PERCENT,200./3.)
        for i in range(30):
            self.lift.observe(1.,self.q,i*.1)
        self.assertFalse(self.lift.moving)
        self.assertEqual(self.lift.snapshot(3.)['phase'],'waiting_neutral')
        self.lift.observe(DEADBAND,self.q,3.)
        self.assertTrue(self.lift.snapshot(3.)['armed'])

    def test_one_second_threshold_and_same_direction_magnitude_changes(self):
        self.lift.observe(0.,self.q,0.)
        self.lift.observe(.21,self.q,.1)
        for i in range(1,10):
            self.lift.observe(.9 if i%2 else .3,self.q,.1+i*.1)
        self.assertFalse(self.lift.moving)
        self.lift.observe(.4,self.q,1.099)
        self.assertFalse(self.lift.moving)
        self.lift.observe(.4,self.q,1.1)
        self.assertTrue(self.lift.moving)
        self.assertEqual(self.lift.snapshot(1.1)['target_percent'],100.)
        # No initial jump at the trigger instant.
        output=self.lift.sample(self.q,1.1)
        for a,b in zip(output,self.q):
            self.assertAlmostEqual(a,b,places=12)

    def test_short_hold_neutral_cancels_elapsed(self):
        self.lift.observe(0.,self.q,0.)
        for i in range(8): self.lift.observe(1.,self.q,.1+i*.1)
        self.lift.observe(0.,self.q,.9)
        self.assertEqual(self.lift.snapshot(.9)['hold_elapsed_s'],0.)
        for i in range(10): self.lift.observe(1.,self.q,1.+i*.1)
        self.assertFalse(self.lift.moving)
        self.lift.observe(1.,self.q,2.)
        self.assertTrue(self.lift.moving)

    def test_sign_reversal_restarts_hold_and_selects_lower_preset(self):
        self.lift.observe(0.,self.q,0.)
        for i in range(8): self.lift.observe(1.,self.q,.1+i*.1)
        for i in range(10): self.lift.observe(-1.,self.q,.9+i*.1)
        self.assertFalse(self.lift.moving)
        self.lift.observe(-1.,self.q,1.9)
        self.assertTrue(self.lift.moving)
        self.assertAlmostEqual(self.lift.snapshot(1.9)['target_percent'],200./3.)

    def test_missing_observations_never_count_as_continuous_hold(self):
        self.lift.observe(0.,self.q,0.)
        self.lift.observe(1.,self.q,.1)
        self.lift.observe(1.,self.q,1.2)
        self.assertFalse(self.lift.moving)
        self.assertEqual(self.lift.snapshot(1.2)['hold_elapsed_s'],0.)
        for i in range(1,11):self.lift.observe(1.,self.q,1.2+i*.1)
        self.assertTrue(self.lift.moving)
        self.assertEqual(MAX_OBSERVATION_GAP_S,.25)

    def test_duplicate_time_does_not_advance_gesture(self):
        self.lift.observe(0.,self.q,0.)
        for _ in range(100): self.lift.observe(1.,self.q,.1)
        self.assertFalse(self.lift.moving)
        self.assertEqual(self.lift.snapshot(.1)['hold_elapsed_s'],0.)

    def test_batched_direction_change_restarts_hold_even_if_latest_axis_same(self):
        self.lift.observe(0.,self.q,0.,gesture_id=1)
        for i in range(9):
            self.lift.observe(1.,self.q,.1+i*.1,gesture_id=2)
        self.assertFalse(self.lift.moving)
        # The SDK saw neutral (3) then up (4); the receiver sees only up.
        self.lift.observe(1.,self.q,1.,gesture_id=4)
        self.assertEqual(self.lift.snapshot(1.)['hold_elapsed_s'],0.)
        for i in range(1,10):
            self.lift.observe(1.,self.q,1.+i*.1,gesture_id=4)
        self.assertFalse(self.lift.moving)
        self.lift.observe(1.,self.q,2.,gesture_id=4)
        self.assertTrue(self.lift.moving)

    def test_gesture_changes_do_not_cancel_motion_or_rearm_without_neutral(self):
        start=self.trigger(1.)
        duration=self.lift.snapshot(start)['duration_s']
        self.lift.observe(-1.,self.q,start+.1,gesture_id=20)
        self.lift.observe(1.,self.q,start+.2,gesture_id=22)
        self.assertTrue(self.lift.moving)
        self.assertEqual(self.lift.snapshot(start+.2)['duration_s'],duration)
        self.assertEqual(self.lift.snapshot(start+.2)['target_percent'],100.)
        q=self.finish(self.q,start)
        now=start+duration+.1
        for i in range(20):
            self.lift.observe(-1.,q,now+i*.1,gesture_id=30+i)
        self.assertFalse(self.lift.moving)
        self.assertFalse(self.lift.snapshot(now+2.)['armed'])
        self.lift.reset()
        self.assertIsNone(self.lift._last_gesture)
        self.lift.observe(1.,q,now+3.,gesture_id=99)
        self.assertEqual(self.lift.snapshot(now+3.)['phase'],'waiting_neutral')

    def test_neutral_after_trigger_continues_without_new_request_sampling(self):
        start=self.trigger(1.)
        self.lift.observe(0.,self.q,start+.01)
        middle=self.lift.sample(self.q,start+self.lift.snapshot(start)['duration_s']/2)
        self.assertGreater(self.model.state(middle)['height'],70.)
        final=self.finish(middle,start)
        self.assertAlmostEqual(self.model.state(final)['height'],100.,places=7)
        self.assertFalse(self.lift.moving)
        self.assertEqual(self.lift.snapshot(start+20)['phase'],'completed')

    def test_single_trigger_and_motion_does_not_queue_opposite_gesture(self):
        start=self.trigger(1.)
        original=self.lift.snapshot(start)
        for i in range(1,11):
            self.lift.observe(-1.,self.q,start+i*.1)
        self.assertEqual(self.lift.snapshot(start+1.)['target_percent'],100.)
        self.assertEqual(self.lift.snapshot(start+1.)['duration_s'],original['duration_s'])
        final=self.finish(self.q,start)
        now=start+original['duration_s']+.1
        for i in range(20):self.lift.observe(-1.,final,now+i*.1)
        self.assertFalse(self.lift.moving)
        self.assertFalse(self.lift.snapshot(now+2.)['armed'])
        self.lift.observe(0.,final,now+2.)
        for i in range(11):self.lift.observe(-1.,final,now+2.1+i*.1)
        self.assertTrue(self.lift.moving)
        self.assertAlmostEqual(self.lift.snapshot(now+3.1)['target_percent'],200./3.)

    def test_raised_lower_preset_holds_until_trigger_and_rises_from_below(self):
        for initial_height in (25.,50.):
            with self.subTest(initial_height=initial_height):
                self.lift.reset()
                q=self.pose(initial_height)
                self.assertEqual(self.lift.sample(q,0.),q)
                self.lift.observe(0.,q,.1)
                self.assertEqual(self.lift.sample(q,.2),q)
                start=self.trigger(-1.,q)
                for actual,expected in zip(self.lift.sample(q,start),q):
                    self.assertAlmostEqual(actual,expected,places=12)
                final=self.finish(q,start)
                height_mm=self.model.state(final)['height_mm']
                old_low_mm=self.model.travel*.5
                self.assertAlmostEqual((height_mm-old_low_mm)/(self.model.travel-old_low_mm),1./3.,places=7)

    def test_one_mm_tolerance_does_not_move_or_retrigger(self):
        q=self.pose(LOW_PERCENT-.5/self.model.travel*100)
        start=self.trigger(-1.,q)
        self.assertFalse(self.lift.moving)
        self.assertEqual(self.lift.snapshot(start)['phase'],'at_target')
        self.assertEqual(self.lift.sample(q,start),q)
        for i in range(20):self.lift.observe(-1.,q,start+i*.1)
        self.assertFalse(self.lift.moving)
        self.assertEqual(TARGET_TOLERANCE_MM,1.)

    def test_pause_clears_motion_and_next_session_requires_neutral(self):
        start=self.trigger(1.)
        q=self.lift.sample(self.q,start+.5)
        self.lift.reset()
        self.assertEqual(self.lift.sample(q,start+1.),q)
        for i in range(20):self.lift.observe(1.,q,start+1.+i*.1)
        self.assertFalse(self.lift.moving)
        self.lift.observe(0.,q,start+3.)
        self.assertTrue(self.lift.snapshot(start+3.)['armed'])

    def test_current_offset_pitch_and_other_axes_are_preserved_during_motion(self):
        q=self.pose(60.,forward=11.,yaw=14.)
        q[18]+=math.radians(.4);q[17]-=math.radians(.4)
        initial=self.model.state(q);start=self.trigger(1.,q)
        self.assertTrue(self.lift.moving)
        duration=self.lift.snapshot(start)['duration_s']
        for i in range(251):
            commanded=q[:]
            # Arm motion may continue independently while torso moves.
            commanded[0]+=.001*i
            commanded[14]-=.0001*i
            output=self.lift.sample(commanded,start+duration*i/250)
            state=self.model.state(output)
            self.assertAlmostEqual(state['forward_offset_mm'],initial['forward_offset_mm'],delta=1e-6)
            self.assertAlmostEqual(state['forward'],initial['forward'],places=8)
            for joint in range(20):
                if joint not in FOLD:self.assertEqual(output[joint],commanded[joint])
        self.assertLessEqual(self.model.state(output)['height'],100.+1e-7)
        self.assertGreaterEqual(math.degrees(output[18]),self.model.config['high_knee_deg']-1e-7)

    def test_full_up_down_finite_continuous_and_strict_velocity_bound(self):
        q=self.pose(50.,forward=4.)
        for turn,direction in enumerate((1.,-1.)):
            self.lift.reset();start=self.trigger(direction,q,start=10.+turn*50.)
            points=self.lift._points
            self.assertEqual(len(points),101)
            for node in points:
                for field in ('positions','velocities','accelerations'):
                    self.assertTrue(all(math.isfinite(x) for x in node[field]))
            for endpoint in (points[0],points[-1]):
                self.assertLess(max(abs(v) for v in endpoint['velocities']),1e-10)
                self.assertLess(max(abs(v) for v in endpoint['accelerations']),1e-10)
            # Bezier control-point derivative bound is conservative over each
            # entire polynomial piece, not just sampled times.
            for left,right in zip(points,points[1:]):
                dt=right['time']-left['time']
                for j in FOLD:
                    b=bezier(left['positions'][j],left['velocities'][j],left['accelerations'][j],
                             right['positions'][j],right['velocities'][j],right['accelerations'][j],dt)
                    bound=max(abs(5*(y-x)/dt) for x,y in zip(b,b[1:]))
                    self.assertLessEqual(math.degrees(bound),30.+1e-7)
            duration=points[-1]['time'];previous=q[:]
            for i in range(1,1001):
                output=self.lift.sample(q,start+duration*i/1000)
                average=max(abs(math.degrees(output[j]-previous[j])) for j in FOLD)/(duration/1000)
                self.assertLessEqual(average,30.+1e-6)
                previous=output
            q=self.lift.sample(output,start+duration+.001)
            self.assertAlmostEqual(self.model.state(q)['height'],100. if direction>0 else 200./3.,places=7)

    def test_high_target_respects_current_offset_reachable_knee_limit(self):
        q=self.pose(60.)
        q[18]+=math.radians(3.);q[17]-=math.radians(3.)
        start=self.trigger(1.,q)
        status=self.lift.snapshot(start)
        self.assertLess(status['effective_target_percent'],100.)
        output=self.finish(q,start)
        self.assertAlmostEqual(math.degrees(output[18]),15.,places=7)
        self.assertAlmostEqual(self.model.state(output)['height'],status['effective_target_percent'],places=7)

    def test_large_roll_cannot_descend_without_changing_held_axes(self):
        q=self.pose(100.,roll=10.)
        with self.assertRaisesRegex(ValueError,'侧倾'):
            self.trigger(-1.,q)
        self.assertFalse(self.lift.moving)
        self.assertFalse(self.lift.snapshot(12.)['armed'])

    def test_small_encoder_noise_is_preserved_not_corrected(self):
        q=self.pose(70.)
        q[17]-=math.radians(.02);q[16]=math.radians(.03)
        start=self.trigger(-1.,q)
        output=self.finish(q,start)
        self.assertAlmostEqual(self.model.state(output)['forward'],-.02,places=8)
        self.assertEqual(output[16],q[16])

    def test_invalid_axis_command_time_and_calibration_fail_closed(self):
        for axis in (True,float('nan'),float('inf'),1.01,'up'):
            self.lift.reset()
            with self.assertRaises(ValueError):self.lift.observe(axis,self.q,0.)
            self.assertFalse(self.lift.moving)
        for invalid in ([0.]*19,[float('nan')]*20):
            with self.assertRaises(ValueError):self.lift.observe(0.,invalid,0.)
        with self.assertRaisesRegex(ValueError,'参考未标定'):
            PresetLiftPlanner({**self.model.config,'upright_confirmed':False})
        start=self.trigger(1.)
        with self.assertRaisesRegex(ValueError,'不能倒退'):
            self.lift.sample(self.q,start-.1)
        self.assertFalse(self.lift.moving)
        for invalid in (True,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.lift.observe(0.,self.q,invalid)

    def test_final_target_is_returned_once_even_after_a_long_sampling_gap(self):
        start=self.trigger(-1.)
        self.assertTrue(self.lift.moving)
        final=self.lift.sample(self.q,start+30.)
        self.assertFalse(self.lift.moving)
        self.assertAlmostEqual(self.model.state(final)['height'],200./3.,places=7)
        self.assertEqual(self.lift.sample(final,start+31.),final)


if __name__=='__main__':
    unittest.main()
