"""Read-only ROS graph and live battery telemetry check in the selected domain."""
import json
import os
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('battery_topic_monitor_audit')
records = {}
subscriptions = []
errors = {}
start = time.monotonic()
try:
    while time.monotonic() - start < 4:
        rclpy.spin_once(node, timeout_sec=.1)
    topics = node.get_topic_names_and_types()
    candidates = [(name, types) for name, types in topics
                  if any(word in (name + ' ' + ' '.join(types)).lower()
                         for word in ['battery', 'bms', '电池', '电量'])]
    for name, types in candidates:
        records[name] = {'types': types, 'messages': 0, 'samples': []}
        if len(types) != 1:
            errors[name] = 'Ambiguous ROS message type'
            continue
        try:
            msg_type = get_message(types[0])
            def callback(msg, topic=name):
                r = records[topic]
                r['messages'] += 1
                sample = message_to_ordereddict(msg)
                if len(r['samples']) < 3:
                    r['samples'].append(sample)
                r['last_sample'] = sample
            subscriptions.append(node.create_subscription(msg_type, name, callback, qos_profile_sensor_data))
        except Exception as exc:
            errors[name] = repr(exc)
    end = time.monotonic() + 12
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    def endpoint(info):
        return {'node_name': info.node_name, 'node_namespace': info.node_namespace,
                'topic_type': info.topic_type, 'qos_reliability': str(info.qos_profile.reliability),
                'qos_durability': str(info.qos_profile.durability)}
    for name in set(records) | {'/battery_state'}:
        record = records.setdefault(name, {'messages': 0, 'samples': []})
        record['publishers'] = [endpoint(i) for i in node.get_publishers_info_by_topic(name)]
        record['subscriptions'] = [endpoint(i) for i in node.get_subscriptions_info_by_topic(name)
                                   if i.node_name != node.get_name()]
    print(json.dumps({'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                      'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
                      'discovery_seconds': 4, 'sample_seconds': 12,
                      'topics': topics, 'battery': records,
                      'nodes': node.get_node_names_and_namespaces(), 'errors': errors},
                     ensure_ascii=False), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
