"""Resolve "auto" hand ports to stable /dev/serial/by-id paths.

Each A3.2 enumerates as an STM32 virtual COM port with a unique USB serial.
The firmware side reply (1 right, 2 left) is optional and not always sent, so
a side learned once is remembered per USB serial in .local/dxhand/hand_ports.json.
Resolution never sends motor commands; probing only connects and reads.
"""
import json
import os
from pathlib import Path

BY_ID = Path("/dev/serial/by-id")
MARK = "STM32_Virtual_ComPort"
SIDES = {1: "right", 2: "left"}


def devices(by_id=BY_ID):
    """{usb_serial: by-id path} for every attached A3.2 controller."""
    if not by_id.is_dir():
        return {}
    found = {}
    for path in sorted(by_id.iterdir()):
        name = path.name
        if MARK in name:
            serial = name.split(MARK + "_", 1)[-1].rsplit("-if", 1)[0]
            found[serial] = str(path)
    return found


def load(store):
    try:
        data = json.loads(Path(store).read_text())
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in data.items() if isinstance(k, str) and v in ("left", "right")} \
        if isinstance(data, dict) else {}


def remember(store, serial, side):
    data = load(store)
    if data.get(serial) == side:
        return
    # A serial maps to one side, and a side to one serial (hand replaced).
    data = {k: v for k, v in data.items() if v != side}
    data[serial] = side
    store = Path(store)
    store.parent.mkdir(parents=True, exist_ok=True)
    tmp = store.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True))
    os.replace(tmp, store)


def serial_of(path, by_id=BY_ID):
    real = os.path.realpath(path)
    for serial, link in devices(by_id).items():
        if os.path.realpath(link) == real:
            return serial
    return None


def resolve(ports, store, probe, by_id=BY_ID):
    """Replace "auto" values in {side: port}. probe(path) -> firmware side code or None."""
    wanted = [side for side, port in ports.items() if port == "auto"]
    if not wanted:
        return dict(ports)
    attached = devices(by_id)
    taken = {os.path.realpath(p) for p in ports.values() if p != "auto"}
    free = {s: p for s, p in attached.items() if os.path.realpath(p) not in taken}
    if not free:
        raise RuntimeError("未找到达显手 USB 设备（/dev/serial/by-id/*STM32_Virtual_ComPort*）")
    known = load(store)
    sides = {serial: known[serial] for serial in free if serial in known}
    for serial, path in free.items():
        if serial in sides or set(wanted) <= set(sides.values()):
            continue
        side = SIDES.get(probe(path))
        if side:
            sides[serial] = side
            remember(store, serial, side)
    result = dict(ports)
    for side in wanted:
        matches = [free[s] for s, v in sides.items() if v == side]
        if len(matches) == 1:
            result[side] = matches[0]
        elif len(wanted) == 1 and len(free) == 1 and not sides:
            result[side] = next(iter(free.values()))  # One unknown hand: use it.
        else:
            raise RuntimeError(
                f"无法自动识别{'左' if side == 'left' else '右'}手 USB（已连接 {len(free)} 个）："
                "固件未回复手别且无记录。请给该手断电重启后再试，或手动填写 USB 路径")
    return result
