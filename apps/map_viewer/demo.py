"""Deterministic, explicitly simulated map data. No SSH, ROS or robot clients."""
import base64
import math
import threading
import time
from types import SimpleNamespace
import zlib

import numpy as np


class DemoStream:
    def __init__(self):
        self.args = SimpleNamespace(host='offline', user='demo', check=False, demo=True)
        self.control_password = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=lambda: None)
        self.scenario = 'tracking'
        self.started = time.monotonic()
        self.elapsed = None  # Set explicitly in replay/screenshot tests.
        self.grid = self.make_map()
        rng = np.random.default_rng(20261023)
        self.particle_noise = rng.normal(size=(460, 3))
        cells = np.frombuffer(zlib.decompress(base64.b64decode(self.grid['data'])), dtype=np.int8)
        occupied = cells.reshape(self.grid['height'], self.grid['width']) == 100
        free = cells.reshape(self.grid['height'], self.grid['width']) == 0
        boundary = occupied & (np.roll(free, 1, 0) | np.roll(free, -1, 0) |
                               np.roll(free, 1, 1) | np.roll(free, -1, 1))
        rows, cols = np.nonzero(boundary)
        self.wall_points = np.column_stack((cols[::2]*.05-4, rows[::2]*.05-3))

    @staticmethod
    def make_map():
        cells = np.full((220, 280), -1, dtype=np.int8)
        cells[16:204, 16:264] = 0
        def box(x0, y0, x1, y1, value=100):
            a,b,c,d = [int(round(n)) for n in ((x0+4)/.05, (y0+3)/.05, (x1+4)/.05, (y1+3)/.05)]
            cells[b:d, a:c] = value
        for rect in [(-3.2,-2.2,9.2,-2.05), (-3.2,7.05,9.2,7.2),
                     (-3.2,-2.2,-3.05,7.2), (9.05,-2.2,9.2,7.2),
                     (-.9,-2.2,-.75,.2), (-.9,1.4,-.75,4.0), (-.9,5.2,-.75,7.2),
                     (-3.2,2.5,-.8,2.65), (5.5,5.25,9.2,5.4), (5.5,5.25,5.65,7.2),
                     (1.2,-.65,3.4,.05), (6.8,-.9,8.1,.1),
                     (1.3,5.6,3.6,6.3), (-2.65,-1.35,-1.35,-.7),
                     (-2.65,3.5,-1.4,4.5), (7.25,5.95,8.3,6.65)]:
            box(*rect)
        return {'type':'map', 'frame':'map', 'width':280, 'height':220,
                'resolution':.05, 'origin':[-4.,-3.,0.], 'stamp':1000.,
                'data':base64.b64encode(zlib.compress(cells.tobytes())).decode()}

    def robot_pose(self):
        seconds = self.elapsed if self.elapsed is not None else time.monotonic()-self.started+24
        angle = seconds/48
        return [3.5+2.25*math.cos(angle), 2.55+1.3*math.sin(angle),
                math.atan2(1.3*math.cos(angle), -2.25*math.sin(angle))]

    def snapshot(self):
        now = time.monotonic()
        pose = self.robot_pose()
        candidate = self.scenario == 'candidate'
        empty = self.scenario == 'empty'
        sigma = .38 if candidate else .045
        pts = self.particle_noise * [sigma, sigma*.72, .30 if candidate else .06]
        pts += np.asarray(pose)
        weights = np.exp(-.5*np.sum(self.particle_noise[:,:2]**2, axis=1))
        particles = np.column_stack((pts, weights)).tolist()
        walls = self.wall_points
        delta = walls-np.asarray(pose[:2])
        near = np.linalg.norm(delta, axis=1)<5
        walls, delta = walls[near], delta[near]
        c,s = math.cos(pose[2]), math.sin(pose[2])
        local = delta @ np.array([[c,-s],[s,c]])
        front = local[:,0]>=0
        topics = {t:{'age':.04,'hz':15. if 'scan' in t else 30.,
                      'latest':{'position_variance':.002,'vx':.04,'vy':0.,'wz':.01}}
                  for t in ['/front/scan','/back/scan','/scan','/odom','/eight_dof_chassis_controller/odom']}
        topics['/map'] = {'age':1.,'hz':0.}
        loc = {'active':not empty,'id':'offline-session','initial_pose_stamp':1000.,
               'ready':not(candidate or empty), 'estimate_visible':not empty,
               'state':'uncertain' if candidate else 'stopped' if empty else 'tracking',
               'reason':'模拟候选分布，尚未收敛' if candidate else '离线模拟定位',
               'pose':pose,'xy_std':sigma,'yaw_std':.30 if candidate else .06,
               'covariance_xy':[sigma*sigma, sigma*sigma*.15, sigma*sigma*.52],
               'matches':36,'stamp':time.time()-.1}
        route = self.preview_path({'x':6.5,'y':4.3,'yaw':0.})
        local_route = [[pose[0]+i*.035*math.cos(pose[2]), pose[1]+i*.035*math.sin(pose[2])]
                       for i in range(16)]
        frame = {'type':'frame','time':time.time(),'frame':'map','demo':True,
                 'map_mode':'empty' if empty else 'saved', 'saved_map_source':None if empty else 'demo-lab / map.yaml',
                 'saved_map_modified':1000., 'mapping_paused':False,'pose':None if empty else pose,
                 'tf_error':'尚未定位' if empty else None,'topics':topics,'localization':loc,
                 'odometry_mode':'raw','odometry_source':'/eight_dof_chassis_controller/odom',
                 'odometry_status':{'running':True,'epoch':'demo-odom'},
                 'display_session':['empty' if empty else 'saved', None if empty else 'demo-lab / map.yaml',
                                    None, None if empty else 'offline-session',
                                    None if empty else 1000., 'demo-odom'],
                 'particles':{'points':[] if empty else particles,'count':460,'valid':not empty,
                              'age':.1,'state':'fresh','reason':'模拟粒子 · 无实机连接'},
                 'navigation_paths':{
                     'global':{'points':route,'age':.2,'valid':not(candidate or empty),'reason':'模拟控制器路径'},
                     'local':{'points':local_route,'age':.1,'valid':not(candidate or empty),'reason':'模拟局部预测'}},
                 'scans':{} if empty else {'/front/scan':walls[front].tolist(),'/back/scan':walls[~front].tolist()},
                 'local_scans':{'/front/scan':local[front].tolist(),'/back/scan':local[~front].tolist()}}
        state = {'frame':frame, 'received':now-5 if self.scenario=='stale' else now}
        if not empty:
            state['map'] = self.grid
        return state

    def preview_path(self, pose):
        """Display-only curve, deliberately not a collision-checked plan."""
        start = np.asarray(self.robot_pose()[:2]);end = np.asarray([pose['x'],pose['y']])
        return [(start+(end-start)*t+np.array([-.18,.2])*math.sin(math.pi*t)).tolist()
                for t in np.linspace(0,1,60)]
