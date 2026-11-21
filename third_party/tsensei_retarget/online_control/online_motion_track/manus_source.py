"""One Manus SDK owner, shared by both hands and all browser sessions."""
from copy import deepcopy
import importlib
from pathlib import Path
import sys
import threading
import time
import uuid

import numpy as np

STALE_AFTER_MS = 200
MODULE_DIR = Path(__file__).resolve().parents[2] / 'vendor/manussdk/build/python'
# The native binding has one process-global callback owner, even across Applications.
SDK_OWNER = threading.Lock()
FINGER_SPEC = [(5, joint) for joint in (1, 2, 4, 5)] + [
    (chain, joint) for chain in (6, 7, 8, 9) for joint in (2, 3, 4, 5)]


def mediapipe_rows(nodes):
    """Map semantic SDK nodes to wrist, thumb, index, middle, ring, pinky."""
    ids = {int(node.node_id) for node in nodes}
    roots = [i for i, node in enumerate(nodes) if
             int(node.parent_id) == int(node.node_id) or int(node.parent_id) not in ids]
    hand_roots = [i for i in roots if int(nodes[i].chain_type) == 13]
    roots = hand_roots or roots
    if len(ids) != len(nodes) or len(roots) != 1:
        raise ValueError('Manus 骨架缺少唯一腕部根节点')
    lookup = {}
    for row, node in enumerate(nodes):
        key = (int(node.chain_type), int(node.finger_joint_type))
        if key in FINGER_SPEC:
            if key in lookup:
                raise ValueError(f'Manus 骨架关节重复：{key}')
            lookup[key] = row
    missing = [key for key in FINGER_SPEC if key not in lookup]
    if missing:
        raise ValueError(f'Manus 骨架缺少手指关节：{missing}')
    return [roots[0]] + [lookup[key] for key in FINGER_SPEC]


def hand_frame(snapshot, nodes, side):
    """Global metre positions -> wrist-relative 21 points; keep handedness."""
    skeleton = np.asarray(snapshot['skeleton'], dtype=float)
    if skeleton.shape != (len(nodes), 10) or not np.isfinite(skeleton).all():
        raise ValueError('Manus 需要有限的 N×10 骨架与对应节点信息')
    rows = mediapipe_rows(nodes)
    points = skeleton[rows, :3].copy()
    points -= points[0]
    if np.max(np.abs(points)) > 10:
        raise ValueError('Manus 手部相对坐标超出 10 米范围')
    raw = {key: value for key, value in snapshot.items() if key != 'skeleton'}
    raw.update(schema='manus.raw_skeleton.v1', side=side, units='m',
        coordinate_space='manus_world', skeleton=skeleton.tolist(), joint_mapping=rows,
        node_info=[{key: int(getattr(node, key)) for key in
                    ('node_id', 'parent_id', 'chain_type', 'finger_joint_type')} for node in nodes])
    hand = dict(world=points.tolist(), normalized=points.tolist(), label=side.title(),
        source='manus', coordinate_space='manus_wrist_relative', normalized_is_image=False)
    return raw, hand


def load_sdk(module_dir=None):
    directory = str(Path(module_dir or MODULE_DIR).expanduser().resolve())
    if directory not in sys.path:
        sys.path.insert(0, directory)
    try:
        return importlib.import_module('manus_glove')
    except (ImportError, OSError) as exc:
        raise ImportError(f'请用运行服务的 Python 执行 scripts/build_manus.sh；Manus SDK 加载失败：{exc}') from exc


class ManusSource:
    def __init__(self, *, mode='integrated', module_dir=None, left_calibration=None,
                 right_calibration=None, connect_timeout=10, clock=time.monotonic,
                 sdk_loader=None, retry_delay=1.):
        if mode not in ('integrated', 'local', 'remote'):
            raise ValueError('Manus mode 必须为 integrated、local 或 remote')
        if type(connect_timeout) is not int or not 1 <= connect_timeout <= 60:
            raise ValueError('Manus connect_timeout 必须在 1..60 秒')
        self.mode, self.connect_timeout = mode, connect_timeout
        self.calibrations = {side: Path(path).expanduser().resolve() for side, path in
            (('left', left_calibration), ('right', right_calibration)) if path is not None}
        for path in self.calibrations.values():
            if not path.is_file():
                raise ValueError(f'Manus 标定文件不存在：{path}')
        self.clock, self.retry_delay = clock, retry_delay
        self.sdk_loader = sdk_loader or (lambda: load_sdk(module_dir))
        self.lock = threading.RLock()
        self.worker = self.on_change = None
        self.stream_id = None
        self.revision = 0
        self.sides = {side: self._empty() for side in ('left', 'right')}

    @staticmethod
    def _empty():
        return dict(revision=0, epoch=0, sequence=-1, glove_id=0, observed_at=None,
                    raw=None, hand=None, event='waiting', error=None)

    def _notify(self):
        if self.on_change:
            self.on_change()

    def _current(self, worker):
        return self.worker is worker and not worker['stop'].is_set()

    def _clear(self, worker, event, error=None, side=None):
        changed = False
        with self.lock:
            if not self._current(worker):
                return
            for key in (side,) if side else ('left', 'right'):
                state = self.sides[key]
                if state['event'] == event and state['error'] == error and state['hand'] is None:
                    continue
                self.revision += 1
                state.update(hand=None, raw=None, observed_at=None, event=event, error=error,
                             revision=self.revision, epoch=state['epoch']+1)
                changed = True
        if changed:
            self._notify()

    def _publish(self, worker, side, snapshot, nodes):
        sequence, arrival = int(snapshot['sequence']), int(snapshot['host_arrival_ns'])
        now = self.clock()
        if not snapshot['valid'] or sequence <= 0 or arrival <= 0 or arrival > int(now * 1e9):
            raise ValueError('Manus 骨架或回调时间无效')
        with self.lock:
            state = self.sides[side]
            if not self._current(worker) or (snapshot['glove_id'] == state['glove_id'] and
                                            sequence <= state['sequence']):
                return
        raw, hand = hand_frame(snapshot, nodes, side)
        with self.lock:
            if not self._current(worker):
                return
            state = self.sides[side]
            if (state['hand'] is None or state['glove_id'] != snapshot['glove_id'] or
                    (now - state['observed_at']) * 1000 >= STALE_AFTER_MS):
                state['epoch'] += 1
            self.revision += 1
            state.update(revision=self.revision, sequence=sequence, glove_id=snapshot['glove_id'],
                observed_at=arrival / 1e9, raw=raw, hand=hand, event='tracking', error=None)
        self._notify()

    def _run(self, worker):
        while not worker['stop'].is_set():
            if not SDK_OWNER.acquire(timeout=.1):
                continue
            glove = None
            try:
                if not self._current(worker):
                    return
                sdk = self.sdk_loader()
                glove = sdk.ManusGlove()
                if not hasattr(glove, 'get_raw_skeleton_snapshot'):
                    raise ImportError('Manus 绑定缺少原子帧接口，请重新编译 vendor/manussdk')
                self._clear(worker, 'connecting')
                if not glove.connect(mode=self.mode, world_coordinates=True, timeout_seconds=self.connect_timeout):
                    raise RuntimeError('Manus 连接失败，请检查 USB 接收器或 Manus Core')
                if not self._current(worker):
                    return
                if hasattr(glove, 'set_recording_enabled'):
                    glove.set_recording_enabled(False)
                with self.lock:
                    self.stream_id = uuid.uuid4().hex
                    for state in self.sides.values():
                        state['sequence'] = -1
                mappings, calibrated, calibration_errors = {}, set(), {}
                self._clear(worker, 'connected')
                while not worker['stop'].is_set():
                    if not glove.is_connected():
                        raise RuntimeError('Manus SDK 已断开，正在重连')
                    for side in ('left', 'right'):
                        try:
                            glove_id = glove.get_glove_id(side)
                            if not glove_id:
                                mappings.pop(side, None)
                                calibrated = {key for key in calibrated if key[0] != side}
                                calibration_errors = {key: value for key, value in calibration_errors.items() if key[0] != side}
                                self._clear(worker, 'glove_missing', side=side)
                                continue
                            if side in self.calibrations and (side, glove_id) not in calibrated:
                                if (side, glove_id) in calibration_errors:
                                    raise RuntimeError(calibration_errors[(side, glove_id)])
                                # Never expose a cached pre-calibration sample after changing the model.
                                result = glove.set_calibration(side, self.calibrations[side].read_bytes())
                                if result != 1:
                                    error = f'Manus {side} 标定加载失败，SDK code={result}；检查文件后重新开始输入'
                                    calibration_errors[(side, glove_id)] = error
                                    raise RuntimeError(error)
                                calibrated.add((side, glove_id))
                                with self.lock:
                                    self.sides[side]['sequence'] = int(glove.get_raw_skeleton_snapshot(side)['sequence'])
                                    self.sides[side]['glove_id'] = glove_id
                                mappings.pop(side, None)
                                self._clear(worker, 'calibrated', side=side)
                            snapshot = dict(glove.get_raw_skeleton_snapshot(side))
                            if not snapshot['valid'] or snapshot['glove_id'] != glove_id:
                                self._clear(worker, 'waiting_for_skeleton', side=side)
                                continue
                            if side not in mappings or mappings[side][0] != glove_id:
                                nodes = glove.get_node_info(side)
                                mediapipe_rows(nodes)
                                mappings[side] = (glove_id, nodes)
                            self._publish(worker, side, snapshot, mappings[side][1])
                        except Exception as exc:
                            self._clear(worker, 'invalid_frame', str(exc), side=side)
                    worker['stop'].wait(.008)
            except (ImportError, OSError) as exc:
                self._clear(worker, 'sdk_unavailable', str(exc))
            except Exception as exc:
                self._clear(worker, 'sdk_error', str(exc))
            finally:
                try:
                    if glove is not None:
                        glove.disconnect()
                        # Destroy the native singleton before another source can acquire it.
                        glove = None
                finally:
                    SDK_OWNER.release()
            worker['stop'].wait(self.retry_delay)

    def _start(self):
        if self.worker is None or not self.worker['thread'].is_alive():
            worker = dict(stop=threading.Event())
            worker['thread'] = threading.Thread(target=self._run, args=(worker,),
                                                name='manus-input', daemon=True)
            self.worker = worker
            worker['thread'].start()

    def _snapshot(self, side):
        state = self.sides[side]
        age = None if state['observed_at'] is None else max(0., (self.clock()-state['observed_at'])*1000)
        stopped = self.worker is None or self.worker['stop'].is_set()
        fresh = not stopped and state['hand'] is not None and age is not None and age < STALE_AFTER_MS
        metadata = dict(source='manus', source_joint_count=21, units='m', native_units='m',
            coordinate_space='manus_world', output_coordinate_space='manus_wrist_relative',
            normalized_is_image=False, stream_id=self.stream_id, source_epoch=state['epoch'],
            sequence=state['sequence'], age_ms=age, stale_after_ms=STALE_AFTER_MS,
            tracking=fresh, tracking_valid=None, stale=not fresh, transport='manus_sdk', mode=self.mode,
            event='stopped' if stopped else state['event'], error=state['error'],
            raw_manus=deepcopy(state['raw']))
        return dict(source='manus', revision=state['revision'], age_ms=age,
            hand=deepcopy(state['hand']) if fresh else None, tracking=fresh, connected=fresh,
            source_metadata=metadata, raw_manus=deepcopy(state['raw']))

    def latest(self, side):
        with self.lock:
            self._start()
            return self._snapshot(side)

    def commit_if_current(self, side, snapshot, action):
        with self.lock:
            current = self._snapshot(side)
            meta = snapshot['source_metadata']
            if (self.worker is None or self.worker['stop'].is_set() or
                    meta['source_epoch'] != current['source_metadata']['source_epoch'] or
                    meta['stream_id'] != self.stream_id or
                    (snapshot['hand'] is not None) != current['tracking']):
                return False
            action()
            return True

    def close(self, *, wait=True):
        with self.lock:
            worker = self.worker
            if worker is None:
                return
            worker['stop'].set()
            for state in self.sides.values():
                self.revision += 1
                state.update(hand=None, raw=None, observed_at=None, event='stopped',
                             revision=self.revision, epoch=state['epoch']+1)
        self._notify()
        if wait and worker['thread'] is not threading.current_thread():
            worker['thread'].join(timeout=self.connect_timeout+5)
