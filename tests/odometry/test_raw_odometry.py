"""Measurement preservation and rejection rules; no ROS or robot required."""
import copy
from types import SimpleNamespace as NS
import unittest

from fairino_slam.odometry.raw_odometry import RawSession, planar_message


def message():
    return NS(header=NS(frame_id='odom', stamp=NS(sec=10, nanosec=123)), child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=1., y=-2., z=0.),
                              orientation=NS(x=0., y=0., z=.6, w=.8)), covariance=[.02]*36),
              twist=NS(twist=NS(linear=NS(x=.1, y=.02, z=0.),
                                angular=NS(x=0., y=0., z=.05)), covariance=[.03]*36))


def extrinsic():
    return NS(translation=NS(x=0., y=0., z=.065), rotation=NS(x=0., y=0., z=0., w=1.))


class RawOdometryTests(unittest.TestCase):
    def test_forward_preserves_measurement_and_original_timestamp(self):
        original = message()
        untouched = copy.deepcopy(original)
        out = planar_message(original, extrinsic())
        self.assertEqual(out.header, original.header)
        self.assertEqual(out.pose, original.pose)
        self.assertEqual(out.twist, original.twist)
        self.assertEqual(out.child_frame_id, 'base_footprint')
        self.assertEqual(original, untouched)
        self.assertIsNot(out.pose, original.pose)

    def test_rejects_unknown_frames_or_extrinsics_instead_of_relabelling(self):
        for parent, child in [('map', 'base_link'), ('odom', 'wheel')]:
            msg = message(); msg.header.frame_id = parent; msg.child_frame_id = child
            with self.assertRaisesRegex(ValueError, '坐标系'):
                planar_message(msg, extrinsic())
        with self.assertRaisesRegex(ValueError, '等待'):
            planar_message(message())
        bad = extrinsic(); bad.translation.x = .1
        with self.assertRaisesRegex(ValueError, '纯高度差'):
            planar_message(message(), bad)

    def test_nonfinite_or_invalid_orientation_is_rejected(self):
        msg = message(); msg.pose.pose.orientation.w = 0.
        with self.assertRaisesRegex(ValueError, '姿态'):
            planar_message(msg, extrinsic())
        msg = message(); msg.pose.covariance[7] = float('nan')
        with self.assertRaisesRegex(ValueError, '非有限'):
            planar_message(msg, extrinsic())

    def test_duplicate_or_reversed_time_never_generates_fresh_output(self):
        session = RawSession()
        msg = message()
        session.accept(msg, extrinsic())
        with self.assertRaisesRegex(ValueError, '重复或倒退'):
            session.accept(msg, extrinsic())
        msg.header.stamp.sec = 9
        with self.assertRaisesRegex(ValueError, '重复或倒退'):
            session.accept(msg, extrinsic())
        msg.header.stamp.sec = 11
        session.accept(msg, extrinsic())

    def test_stationary_samples_do_not_accumulate_pose_or_covariance(self):
        session = RawSession(); msg = message()
        msg.twist.twist.linear.x = msg.twist.twist.linear.y = msg.twist.twist.angular.z = 0.
        for sec in range(10, 131):
            msg.header.stamp.sec = sec
            out = session.accept(msg, extrinsic())
            self.assertEqual(out.pose, msg.pose)


if __name__ == '__main__':
    unittest.main()
