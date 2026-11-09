"""Persistent named map poses; this module never sends robot commands."""
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid

from fairino_slam.paths import PROJECT_ROOT

DEFAULT_PATH = PROJECT_ROOT / '.local/navigation/waypoints.json'


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def map_binding(frame, host, user):
    source, modified = frame.get('saved_map_source'), frame.get('saved_map_modified')
    coordinate_frame = frame.get('frame')
    if (frame.get('map_mode') != 'saved' or not isinstance(source, str) or not source
            or not finite(modified) or modified <= 0 or coordinate_frame != 'map'
            or not isinstance(host, str) or not host or not isinstance(user, str) or not user):
        raise ValueError('请先连接机器人并选用已保存地图，等待地图信息完整')
    return dict(host=host, user=user, source=source, modified=modified, frame=coordinate_frame)


def fresh_frame(snapshot, now=None):
    now = time.monotonic() if now is None else now
    received = snapshot.get('received')
    if ('error' in snapshot or not finite(received) or not 0 <= now-received < 2):
        raise ValueError('地图反馈已过期，不能记录或选择位置点')
    return snapshot.get('frame') or {}


def capture(name, snapshot, host, user, now=None):
    frame = fresh_frame(snapshot, now)
    binding = map_binding(frame, host, user)
    loc, pose = frame.get('localization') or {}, frame.get('pose')
    if not loc.get('active') or not loc.get('ready') or not loc.get('id'):
        raise ValueError('请等待定位就绪后再保存当前位置')
    if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not all(map(finite, pose)):
        raise ValueError('当前机器人位置或朝向无效，未保存')
    name = name.strip()
    if not name or len(name) > 64 or any(ord(c) < 32 for c in name):
        raise ValueError('请输入 1～64 个字符的点名称，不能包含换行或控制字符')
    return dict(id=uuid.uuid4().hex, name=name, map=binding,
                pose=dict(x=float(pose[0]), y=float(pose[1]),
                          yaw=math.atan2(math.sin(pose[2]), math.cos(pose[2]))),
                created=time.time())


class WaypointStore:
    def __init__(self, path=None):
        self.path = Path(path) if path is not None else DEFAULT_PATH

    def _read(self):
        try:
            data = json.loads(self.path.read_text())
        except FileNotFoundError:
            return []
        except (OSError, ValueError) as exc:
            raise ValueError('无法读取已保存点；原文件保留：'+str(exc)) from exc
        def ensure(valid):
            if not valid:
                raise ValueError("invalid waypoint record")
        try:
            ensure(isinstance(data, dict) and data['version'] == 1)
            records = data['points']
            ensure(isinstance(records, list))
            ids, names = set(), set()
            for p in records:
                ensure(isinstance(p['id'], str) and p['id'] and p['id'] not in ids)
                ensure(isinstance(p['name'], str) and 0 < len(p['name']) <= 64)
                ensure(not any(ord(c) < 32 for c in p['name']))
                m = p['map']
                ensure(m == map_binding(dict(map_mode='saved', saved_map_source=m['source'],
                    saved_map_modified=m['modified'], frame=m['frame']), m['host'], m['user']))
                ensure(set(p['pose']) == {'x', 'y', 'yaw'} and all(map(finite, p['pose'].values())))
                ensure(finite(p['created']))
                key = (json.dumps(m, sort_keys=True), p['name'])
                ensure(key not in names)
                ids.add(p['id'])
                names.add(key)
            return records
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('已保存点文件格式无效；原文件保留，请检查 '+str(self.path)) from exc

    def list(self, binding):
        return [p for p in self._read() if p['map'] == binding]

    def _change(self, change):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Multiple panels/processes must not overwrite each other's additions.
        with self.path.with_suffix('.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            records = self._read()
            change(records)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                                 prefix='.waypoints-', delete=False) as stream:
                    temporary = Path(stream.name)
                    json.dump(dict(version=1, points=records), stream, ensure_ascii=False, indent=2,
                              allow_nan=False)
                    stream.write('\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            return records

    def save(self, point):
        def change(records):
            if any(p['map'] == point['map'] and p['name'] == point['name'] for p in records):
                raise ValueError('当前地图已有同名点，请使用其他名称')
            records.append(point)
        return [p for p in self._change(change) if p['map'] == point['map']]

    def delete(self, point_id, binding):
        def change(records):
            found = next((p for p in records if p['id'] == point_id and p['map'] == binding), None)
            if found is None:
                raise ValueError('该位置点已不存在，请刷新列表')
            records.remove(found)
        return [p for p in self._change(change) if p['map'] == binding]
