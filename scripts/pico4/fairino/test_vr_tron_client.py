"""Actual TRON2 planner on the real client loop with in-memory I/O only."""
import unittest
from unittest.mock import patch

import numpy as np
import paths
import test_vr_native_client as native_fixture
from tool_frames import tool_config
from tools.ik_experiments.tron2_art7 import dependencies_available
from vendor_kinematics import VendorKinematics
from vr_tron_ik import TronIK


@unittest.skipUnless(dependencies_available(), 'Optional TRON2 native dependencies/build absent')
class TronClientTests(unittest.TestCase):
    backend = 'tron2'
    frame = staticmethod(native_fixture.NativeClientTests.frame)
    run_frames = native_fixture.NativeClientTests.run_frames
    tcp = staticmethod(native_fixture.NativeClientTests.tcp)
    events = staticmethod(native_fixture.NativeClientTests.events)

    @classmethod
    def setUpClass(cls):
        cls.native = TronIK(tool_config())
        cls.kin = VendorKinematics()

    def test_both_arms_routing_and_release_use_tron_packets(self):
        for mode, remaining in (('first_person', 'right'), ('mirror_same_direction', 'left')):
            with self.subTest(mode=mode):
                result = self.run_frames([self.frame(), self.frame(left_grip=1., right_grip=1.),
                    self.frame(left_grip=1., right_grip=1., position=(.002, 0., 0.)),
                    self.frame(right_grip=1., position=(.003, 0., 0.)), self.frame()], mode=mode)
                self.assertEqual([set(g) for g in result.goals],
                                 [{'left', 'right'}, {'left', 'right'}, {remaining}])
                for packet in result.packets[1:4]:
                    request = packet['request']
                    self.assertEqual(request['ik_backend'], 'tron2')
                    self.assertNotIn('demo_only', request)
                    for arm in request['arms'].values():
                        self.assertTrue(arm['native_ik']['success'])
                        self.assertLessEqual(arm['native_ik']['position_error_mm'], .02)
                        self.assertLessEqual(arm['native_ik']['orientation_error_deg'], .01)
                self.assertIsNone(result.packets[-1]['request'])
                self.assertEqual(result.gui[0]['ik_backend'], 'tron2')
                self.assertIn('receiver_phase', result.gui[0])
                self.assertEqual(self.events(result, 'ik_config')[0]['command_tracking'],
                                 'tron2_qp_external_wrist_checked_ik')

    def test_rotation_freeze_and_continuous_half_turn_keep_input_contract(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.),
            self.frame(right_grip=1., angle=4.), self.frame(right_grip=1., angle=8., primary=True),
            self.frame(right_grip=1., angle=16., primary=True), self.frame(right_grip=1., angle=24.)])
        baseline = self.tcp(result.packets[1], requested=True)
        for packet in result.packets[3:]:
            np.testing.assert_allclose(self.tcp(packet, requested=True)[:3, :3], baseline[:3, :3], atol=1e-10)
        # Use the common mapping test directly; its assertions concern goals,
        # not a specific branch's solution or packet metadata.
        native_fixture.NativeClientTests.test_native_uses_continuous_clutch_at_half_turn(self)

    def test_500hz_loop_uses_same_tcp_linear_and_slerp_buffer(self):
        native_fixture.NativeClientTests.test_500hz_loop_interpolates_tcp_position_and_rotation_before_ik(self)

    def test_guard_off_pauses_without_disabling_guard_and_regrip_resumes(self):
        result = self.run_frames([self.frame(), self.frame(right_grip=1.),
            self.frame(right_grip=1.), self.frame(right_grip=1.), self.frame(),
            self.frame(right_grip=1.)], keys={2: 'G'})
        self.assertTrue(all(packet['guard_enabled'] for packet in result.packets))
        self.assertIsNotNone(result.packets[1]['request'])
        self.assertTrue(all(result.packets[i]['request'] is None for i in (2, 3, 4)))
        self.assertIsNotNone(result.packets[5]['request'])
        self.assertTrue(any('TRON2' in e['reason'] and '必须保持开启' in e['reason']
                            for e in self.events(result, 'input_paused')))

    def test_home_and_auxiliary_controls_keep_shared_gates(self):
        native_fixture.NativeClientTests.test_real_home_generates_joint_requests_without_native_ik(self)
        native_fixture.NativeClientTests.test_lift_only_mixed_motion_and_pause_preserve_auxiliary_gates(self)

    def test_bad_solution_holds_entire_request_and_requires_release(self):
        with patch.object(self.native, 'request', side_effect=ValueError('TRON2 rejected candidate')):
            result = self.run_frames([self.frame(), self.frame(left_grip=1., right_grip=1.),
                self.frame(left_grip=1., right_grip=1., position=(.002, 0., 0.)),
                self.frame(), self.frame(left_grip=1., right_grip=1.)])
        self.assertTrue(all(packet['request'] is None for packet in result.packets))
        # A held Grip after rejection does not repeatedly solve; only regrip
        # after a release produces the second attempted dual-arm request.
        self.assertEqual(len(result.goals), 2)


if __name__ == '__main__':
    unittest.main()
