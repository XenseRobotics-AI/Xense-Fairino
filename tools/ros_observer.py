"""Passive ROS snapshot; creates subscriptions and read-only service clients only."""
import json, math, os, time
from pathlib import Path
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage
from hardware_state_broadcaster_msgs.msg import HardwareState
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init(); node=rclpy.create_node('fairino_handover_observer_'+str(os.getpid()))
report={'ros_domain':os.getenv('ROS_DOMAIN_ID'), 'hardware':None, 'scans':{}, 'transforms':{}, 'upper_estop':None}
subs=[]
def hw(m): report['hardware']=message_to_ordereddict(m)
subs.append(node.create_subscription(HardwareState,'/hardware_states',hw,10))
def scan(topic):
 def cb(m):
  now=time.monotonic(); s=report['scans'].setdefault(topic,{'count':0,'first':now})
  s.update(count=s['count']+1,last=now,frame=m.header.frame_id,points=len(m.ranges),finite_points=sum(math.isfinite(x) for x in m.ranges),scan_time=m.scan_time)
 return cb
if os.getenv('ROS_DOMAIN_ID')=='1':
 for topic in ('/front/scan','/back/scan','/scan'):
  subs.append(node.create_subscription(LaserScan,topic,scan(topic),qos_profile_sensor_data))
 def tf(m):
  for t in m.transforms: report['transforms'][t.child_frame_id]=message_to_ordereddict(t)
 subs.append(node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
 subs.append(node.create_subscription(Int32,'/upper_estop_error_code',lambda m:report.update(upper_estop=m.data),10))
client=node.create_client(ListControllers,'/controller_manager/list_controllers')
future=client.call_async(ListControllers.Request()) if client.wait_for_service(timeout_sec=3) else None
end=time.monotonic()+9
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
if future and future.done():report['controllers']={c.name:c.state for c in future.result().controller}
report['topics']=dict(node.get_topic_names_and_types())
for topic,s in report['scans'].items():
 s['hz']=(s['count']-1)/(s['last']-s['first']) if s['count']>1 else 0
 s['publishers']=node.count_publishers(topic)
if os.getenv('ROS_DOMAIN_ID')=='0':
 report['gpio']={str(i):Path(f'/sys/class/gpio/gpio{i}/value').read_text().strip() for i in (56,57)}
print('OBSERVE_JSON '+json.dumps(report,ensure_ascii=False),flush=True)
node.destroy_node();rclpy.shutdown()
