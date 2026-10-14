#!/usr/bin/env python3
"""Read the existing chassis LaserScan topics; never start or stop drivers."""
import argparse
import json
import math
import os
import signal
import sys
import time

DEVICES = [
    ('20260907-000045', '192.168.192.101', 'front', '/front/scan'),
    ('20260907-000046', '192.168.192.100', 'back', '/back/scan'),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args()
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data

    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    rclpy.init()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    node = rclpy.create_node('ls3_viewer_observer_' + str(os.getpid()))
    subscriptions = []
    state = {serial: {'count': 0, 'first': None, 'last': None}
             for serial, _, _, _ in DEVICES}
    pending = {}
    started = time.monotonic()
    last_sent = 0

    def callback(serial, ip, position, topic):
        def receive(msg):
            now = time.monotonic()
            s = state[serial]
            s['count'] += 1
            s['last'] = now
            if s['first'] is None:
                s['first'] = now
            ranges = [round(float(v), 3)
                      if math.isfinite(v) and msg.range_min <= v <= msg.range_max
                      else None for v in msg.ranges]
            hz = ((s['count'] - 1) / (now - s['first'])
                  if s['count'] > 1 and now > s['first'] else 0)
            pending[serial] = {
                'type': 'scan', 'serial': serial, 'ip': ip,
                'position': position, 'topic': topic,
                'frame_id': msg.header.frame_id, 'model': 'LS3-2027DE',
                'hz': round(hz, 3), 'frames': s['count'], 'points': len(ranges),
                'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                'scan_time_reported': msg.scan_time, 'ranges_m': ranges,
            }
        return receive

    try:
        for serial, ip, position, topic in DEVICES:
            subscriptions.append(node.create_subscription(
                LaserScan, topic, callback(serial, ip, position, topic),
                qos_profile_sensor_data))
        print(json.dumps({'type': 'ready', 'domain': 1, 'mode': 'subscribe_only',
                          'devices': DEVICES}), flush=True)
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.025)
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break
            if now - last_sent >= .15:
                for item in pending.values():
                    print(json.dumps(item, separators=(',', ':')), flush=True)
                pending.clear()
                last_sent = now
            if now - started > 10:
                missing = [position for serial, _, position, _ in DEVICES
                           if state[serial]['last'] is None
                           or now - state[serial]['last'] > 5]
                if missing:
                    raise RuntimeError('常驻雷达话题未更新：' + ', '.join(missing))
    except BrokenPipeError:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary = []
        for serial, ip, position, topic in DEVICES:
            s = state[serial]
            hz = ((s['count'] - 1) / (s['last'] - s['first'])
                  if s['count'] > 1 else 0)
            summary.append({'serial': serial, 'ip': ip, 'position': position,
                            'topic': topic, 'frames': s['count'], 'hz': hz})
        try:
            print(json.dumps({'type': 'summary', 'devices': summary}), flush=True)
        except BrokenPipeError:
            pass


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(json.dumps({'type': 'error', 'message': str(e)}), flush=True)
        sys.exit(1)
