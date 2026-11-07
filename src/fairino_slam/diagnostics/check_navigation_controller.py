#!/usr/bin/env python3
"""Exercise the installed lookahead controller in ROS domain 87, with simulated feedback only."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile')
    parser.add_argument('--seconds', type=float, default=18)
    args = parser.parse_args()
    # No physical robot topics are joined. This process and its controller child
    # both use the same dedicated domain and loopback-only discovery.
    os.environ.update(ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1')
    os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from sensor_msgs.msg import LaserScan
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import FollowPath
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('controller_benchmark')
    profile = yaml.safe_load(Path(args.profile).read_text())
    with tempfile.TemporaryDirectory(prefix='fairino-controller-benchmark-') as directory:
        root = Path(directory)
        params = root/'params.yaml'
        params.write_text(yaml.safe_dump({'fairino_navigation': profile}))
        log = (root/'controller.log').open('w+')
        process = subprocess.Popen([
            'nice', '-n', '10', 'ros2', 'run', 'nav2_controller', 'controller_server', '--ros-args',
            '-r', '__ns:=/fairino_navigation', '--params-file', str(params)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            pose = [0., 0., .3]
            heading_errors = []
            command = [0., 0., 0.]
            received = []
            commands = []
            pubmap = node.create_publisher(OccupancyGrid, '/map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            pubodom = node.create_publisher(Odometry, '/odom', 10)
            pubs = [node.create_publisher(LaserScan, '/'+n+'/scan', qos_profile_sensor_data) for n in ('front','back')]
            tf = TransformBroadcaster(node)
            static = StaticTransformBroadcaster(node)
            fixed = []
            for parent, child, x, y, z, yaw in [
                ('base_footprint','base_link',0,0,.065,0),
                ('base_link','front_laser_link',.239,.175,.1455,math.pi/4),
                ('base_link','back_laser_link',-.239,-.175,.1455,-3*math.pi/4)]:
                t = TransformStamped()
                t.header.frame_id, t.child_frame_id = parent, child
                t.header.stamp = node.get_clock().now().to_msg()
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x),float(y),float(z)
                t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw/2),math.cos(yaw/2)
                fixed.append(t)
            static.sendTransform(fixed)
            grid = OccupancyGrid()
            grid.header.frame_id='map';grid.info.resolution=.05;grid.info.width=grid.info.height=200
            grid.info.origin.position.x=grid.info.origin.position.y=-5.
            grid.info.origin.orientation.w=1.;grid.data=[0]*40000
            # The local costmap uses odom; a static identity map->odom is for this
            # isolated numerical test only, never the live robot's TF tree.
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
            static.sendTransform([*fixed,t])
            pubmap.publish(grid)
            def output(msg):
                command[:] = [msg.linear.x,msg.linear.y,msg.angular.z]
                received.append(time.monotonic());commands.append(list(command))
            sub=node.create_subscription(Twist,'/fairino_navigation/cmd_vel',output,10)
            last=time.monotonic()
            last_scan=0.
            def spin(seconds):
                nonlocal last,last_scan
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    now=time.monotonic();dt=min(now-last,.05);last=now
                    vx,vy,w=command if received and now-received[-1]<.3 else (0.,0.,0.)
                    c,s=math.cos(pose[2]),math.sin(pose[2])
                    pose[0]+=(vx*c-vy*s)*dt;pose[1]+=(vx*s+vy*c)*dt;pose[2]+=w*dt
                    heading_errors.append(abs(math.atan2(math.sin(pose[2]-.3), math.cos(pose[2]-.3))))
                    stamp=node.get_clock().now().to_msg()
                    t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_footprint'
                    t.transform.translation.x,t.transform.translation.y=pose[:2]
                    t.transform.rotation.z,t.transform.rotation.w=math.sin(pose[2]/2),math.cos(pose[2]/2);tf.sendTransform(t)
                    odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
                    odom.pose.pose.position.x,odom.pose.pose.position.y=pose[:2]
                    odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=t.transform.rotation.z,t.transform.rotation.w
                    odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=vx,vy,w;pubodom.publish(odom)
                    if now-last_scan>.08:
                        last_scan=now
                        for name,pub in zip(('front','back'),pubs):
                            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id=name+'_laser_link'
                            scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=2*math.pi/360
                            scan.range_min=.05;scan.range_max=20.;scan.ranges=[float('inf')]*361;pub.publish(scan)
                    rclpy.spin_once(node,timeout_sec=.005)
            def wait(future,seconds=15):
                end=time.monotonic()+seconds
                while not future.done() and time.monotonic()<end:spin(.02)
                if not future.done():raise RuntimeError('benchmark ROS request timed out')
                return future.result()
            client=node.create_client(ChangeState,'/fairino_navigation/controller_server/change_state')
            deadline=time.monotonic()+20
            while not client.service_is_ready() and time.monotonic()<deadline:spin(.1)
            spin(2)
            for transition in (1,3):
                req=ChangeState.Request();req.transition.id=transition
                if not wait(client.call_async(req)).success:raise RuntimeError('lifecycle transition failed')
                spin(1)
            action=ActionClient(node,FollowPath,'/fairino_navigation/follow_path')
            deadline=time.monotonic()+10
            while not action.server_is_ready() and time.monotonic()<deadline:spin(.1)
            goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
            goal.path.header.frame_id='map';goal.path.header.stamp=node.get_clock().now().to_msg()
            for i in range(51):
                p=PoseStamped();p.header=goal.path.header
                p.pose.position.x=.6*i/50;p.pose.position.y=.25*i/50
                angle=.3
                p.pose.orientation.z,p.pose.orientation.w=math.sin(angle/2),math.cos(angle/2)
                goal.path.poses.append(p)
            handle=wait(action.send_goal_async(goal))
            if not handle.accepted:raise RuntimeError('path rejected')
            received.clear();commands.clear()
            result=handle.get_result_async()
            start=time.monotonic()
            while time.monotonic()-start<args.seconds and not result.done():spin(.05)
            done_status=result.result().status if result.done() else None
            if not result.done():wait(handle.cancel_goal_async())
            gaps=[b-a for a,b in zip(received,received[1:])]
            log.flush();log.seek(0);text=log.read()
            print(json.dumps({'domain':87,'motors_operated':False,'command_count':len(received),
                'maximum_gap':max(gaps,default=None),'gaps_over_300ms':sum(g>.3 for g in gaps),
                'missed_loop_warnings':text.count('Control loop missed'),
                'simulated_pose':pose,'action_status':done_status,
                'max_heading_error':max(heading_errors,default=0),
                'max_linear':max((math.hypot(*v[:2]) for v in commands),default=0),
                'max_angular':max((abs(v[2]) for v in commands),default=0)}))
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
            log.close()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
