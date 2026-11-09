"""Planar wheel-odometry adapter. No integration, filtering or motion commands."""
import copy
import math


def planar_message(msg, base_transform=None):
    """Keep measured x/y/yaw, twist, covariance and stamp; project onto footprint.

    This robot's base_link and base_footprint differ only in height. Refuse
    other extrinsics rather than silently relabelling a displaced/rotated base.
    """
    if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_link', 'base_footprint'):
        raise ValueError('原始里程计坐标系不符：需要 odom → base_link/base_footprint')
    p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z,
              *msg.pose.covariance, *msg.twist.covariance]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('原始里程计包含非有限值')
    if abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) > .001 or abs(q.x)+abs(q.y) > .001:
        raise ValueError('原始里程计不是有效的平面姿态')
    if msg.child_frame_id == 'base_link':
        if base_transform is None:
            raise ValueError('等待 base_footprint → base_link 静态 TF')
        b, r = base_transform.translation, base_transform.rotation
        values = [b.x, b.y, b.z, r.x, r.y, r.z, r.w]
        if (not all(math.isfinite(v) for v in values) or abs(b.x)+abs(b.y) > 1e-6
                or abs(r.x)+abs(r.y)+abs(r.z)+abs(abs(r.w)-1) > 1e-6):
            raise ValueError('底盘静态 TF 不是纯高度差，拒绝直接使用平面里程计')
    result = copy.deepcopy(msg)
    result.child_frame_id = 'base_footprint'
    result.pose.pose.position.z = 0.
    return result


class RawSession:
    """Forward each accepted measurement once; never extrapolate an old stamp."""
    def __init__(self):
        self.last_stamp = None

    def accept(self, msg, base_transform=None):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= 0 or (self.last_stamp is not None and stamp <= self.last_stamp):
            raise ValueError('原始里程计时间戳重复或倒退；需重新开启里程计会话')
        output = planar_message(msg, base_transform)
        self.last_stamp = stamp
        return output
