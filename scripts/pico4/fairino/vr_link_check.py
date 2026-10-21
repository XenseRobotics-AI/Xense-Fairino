"""Non-actuating SSH/JSON/IK benchmark, optionally with read-only ROS feedback."""
import json
import math
import queue
import sys
import threading
import time

from vr_core import validate
from vr_timing import Timing, Deadline, RateMeter


def intervals(times):
    values = sorted(1000*(b-a) for a, b in zip(times, times[1:]))
    if not values:
        return {"samples": len(times), "hz": 0.}
    return {"samples": len(times), "hz": (len(times)-1)/(times[-1]-times[0]),
            "median_ms": values[len(values)//2], "p95_ms": values[int(.95*(len(values)-1))],
            "max_ms": values[-1]}


def distribution(values):
    values = sorted(values)
    if not values:
        return {"samples": 0}
    return {"samples": len(values), "min_ms": values[0],
            "median_ms": values[len(values)//2],
            "p95_ms": values[int(.95*(len(values)-1))],
            "p99_ms": values[int(.99*(len(values)-1))], "max_ms": values[-1]}


def run_server(args):
    # Optional ROS is a hardware-state subscriber only. No motion publisher,
    # action client, controller loading or enable operation exists in this path.
    from probe_core import snapshot, JOINTS
    timing = Timing.from_args(args)
    lock, stop = threading.Lock(), threading.Event()
    input_ready = threading.Event()
    latest = [None]
    errors = []
    output = queue.Queue(maxsize=40)
    def read():
        try:
            for line in sys.stdin:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    raise ValueError("无效压测输入")
                packet["server_read"] = time.monotonic()
                with lock:
                    latest[0] = packet
                    input_ready.set()
                if packet.get("exit"):
                    break
        except Exception as exc:
            errors.append(str(exc))
        finally:
            stop.set()
            input_ready.set()
    def write():
        broken = False
        while True:
            value = output.get()
            try:
                if not broken:
                    value["server_write"] = time.monotonic()
                    print("@UPPERPROBE "+json.dumps(value, allow_nan=False), flush=True)
            except (OSError, ValueError):
                broken = True
                errors.append("压测输出连接已断开")
                stop.set()
            finally:
                output.task_done()
    threading.Thread(target=read, daemon=True).start()
    threading.Thread(target=write, daemon=True).start()
    ros_node = None
    hardware_times = []
    if getattr(args, "observe_ros", False):
        import rclpy
        from hardware_state_broadcaster_msgs.msg import HardwareState
        rclpy.init()
        ros_node = rclpy.create_node("fairino_vr_latency_readonly")
        def observe(msg):
            # Same snapshot parsing cost as the real worker, without control APIs.
            snapshot({k: getattr(msg, k) for k in msg.get_fields_and_field_types()})
            hardware_times.append(time.monotonic())
        ros_node.create_subscription(HardwareState, "/hardware_states", observe, 1)
    started = time.monotonic()
    feedback = Deadline(timing.feedback_hz, started)
    meter = RateMeter()
    accepted, skipped = 0, 0
    accepted_times, feedback_times, validation_ms, dispatch_ms = [], [], [], []
    traces = []
    hw = snapshot({"joint_names": JOINTS, "position": [0.]*20,
        "command_position": [0.]*20, "velocity": [0.]*20,
        "stateword": [0x2240]*20, "errorcode": [0]*20, "hexerrorcode_curerrcode": 0})
    while not stop.is_set() and time.monotonic()-started < args.duration+20:
        now = time.monotonic()
        with lock:
            packet = latest[0]
        request = (packet or {}).get("request")
        if request and request.get("id", 0) != accepted:
            try:
                if request["id"] <= accepted:
                    raise ValueError("压测请求序号倒退")
                t = time.monotonic()
                validate(hw, request, now, getattr(args, 'tools', None))
                validated = time.monotonic()
                validation_ms.append(1000*(validated-t))
                dispatch_ms.append(1000*(t-packet["server_read"]))
                traces.append({"id": request["id"], "server_read": packet["server_read"],
                    "server_work": t, "server_validated": validated})
                skipped += max(0, request["id"]-accepted-1)
                accepted = request["id"]
                accepted_times.append(now)
                meter.mark(now)
            except ValueError as exc:
                errors.append(str(exc))
                stop.set()
        if now >= feedback.next:
            feedback_times.append(now)
            try:
                output.put_nowait({"stamp": now, "link_check": True, "synthetic_hardware": True,
                    "hardware": hw, "last_request": accepted, "target_count": meter.total,
                    "target_rate_hz": meter.hz(now), "skipped_targets": skipped,
                    "timing": timing.payload(), "traces": traces})
                traces = []
            except queue.Full:
                errors.append("压测回传队列已满")
                break
            feedback.advance(now)
        timeout = min(1/timing.receiver_hz, max(0., feedback.next-time.monotonic()))
        if ros_node is None:
            input_ready.wait(timeout)
        else:
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(ros_node, timeout_sec=min(.001, max(0., end-time.monotonic())))
                if input_ready.is_set():
                    break
        input_ready.clear()
    if ros_node is not None:
        ros_node.destroy_node()
        rclpy.shutdown()
    final = {"stamp": time.monotonic(), "link_check": True, "last_request": accepted,
        "traces": traces,
        "link_check_summary": {"accepted": intervals(accepted_times),
            "feedback": intervals(feedback_times), "skipped_targets": skipped,
            "validation_max_ms": max(validation_ms, default=0.),
            "validation": distribution(validation_ms), "dispatch_wait": distribution(dispatch_ms),
            "observed_hardware": intervals(hardware_times), "errors": errors}}
    output.put(final, timeout=2.)
    output.join()
    return 1 if errors else 0


def run_client(args, kin, connection):
    from tcp_core import ARMS
    from probe_core import snapshot, JOINTS
    from tcp_keyboard import verify_both
    from vr_home import load_home
    from vr_plan import request
    timing = Timing.from_args(args)
    q_by_side = load_home()["target"]
    q = q_by_side["left"]+q_by_side["right"]+[0.]*6
    hw = snapshot({"joint_names": JOINTS, "position": q[:], "command_position": q[:],
        "velocity": [0.]*20, "stateword": [0x2240]*20,
        "errorcode": [0]*20, "hexerrorcode_curerrcode": 0})
    proof, _ = verify_both(kin, hw)
    deadline = time.monotonic()+10.
    while not connection.latest.get("link_check"):
        if connection.done.is_set() or time.monotonic() > deadline:
            raise RuntimeError("通信压测端未就绪")
        time.sleep(.01)
    print(f"通信压测 {args.duration:g} 秒：模拟双臂目标，未启动 ROS 控制器，电机不动作。只读 ROS 观察={getattr(args, 'observe_ros', False)}", flush=True)
    start = time.monotonic()
    cadence = Deadline(timing.command_hz, start)
    sent_times, feedback_times, acknowledgement_ms = [], [], []
    sent_at, acknowledged = {}, set()
    ik_ms, local_validation_ms, transport_rtt_ms, response_wait_ms = [], [], [], []
    sequence = 0
    outgoing = None
    def events():
        while True:
            try:
                event = connection.events.get_nowait()
            except queue.Empty:
                break
            if event.get("error"):
                raise RuntimeError(event["error"])
            if event.get("stamp") is not None and start <= time.monotonic():
                feedback_times.append(event["stamp"])
            received = event.get("client_received_monotonic", time.monotonic())
            for trace in event.get("traces", []):
                n = trace["id"]
                if n not in sent_at or n in acknowledged:
                    continue
                rtt = received-sent_at[n]
                server_residence = event["server_write"]-trace["server_read"]
                acknowledgement_ms.append(1000*rtt)
                transport_rtt_ms.append(1000*(rtt-server_residence))
                response_wait_ms.append(1000*(event["server_write"]-trace["server_validated"]))
                acknowledged.add(n)
    while time.monotonic()-start < args.duration:
        now = time.monotonic()
        events()
        if connection.done.is_set() or now-connection.last_status > .25:
            raise RuntimeError("通信压测通道中断或反馈超过 250 ms")
        goals = {}
        for side in ARMS:
            angles = [v+.004*math.sin((now-start)*.5+j*.4) for j, v in enumerate(q_by_side[side])]
            goals[side] = kin.forward_tcp(angles, side)[0]
        sequence += 1
        compute_start = time.monotonic()
        outgoing = request(kin, hw, goals, sequence, connection.latest["stamp"])
        ik_ms.append(1000*(time.monotonic()-compute_start))
        validation_start = time.monotonic()
        validate(hw, outgoing, connection.latest["stamp"], getattr(kin, 'tools', None))
        local_validation_ms.append(1000*(time.monotonic()-validation_start))
        sent_at[sequence] = time.monotonic()
        sent_times.append(sent_at[sequence])
        connection.send(request=outgoing, kinematics=proof)
        time.sleep(cadence.advance(time.monotonic()))
    # Drain the final target before closing; no stale command is replayed.
    deadline = time.monotonic()+.2
    while connection.latest.get("last_request", 0) < sequence and time.monotonic() < deadline:
        events()
        time.sleep(.005)
    connection.send(exit=True)
    if not connection.done.wait(5.):
        raise RuntimeError("通信压测端未按时退出")
    events()
    remote = connection.latest.get("link_check_summary", {})
    summary = {"mode": "non_actuating_link_check", "timing": timing.payload(),
        "observe_ros": getattr(args, "observe_ros", False),
        "sent": intervals(sent_times), "remote": remote, "missed_loop_slots": cadence.skipped,
        "acknowledged_targets": len(acknowledged),
        "computer_bimanual_ik": distribution(ik_ms),
        "computer_fk_validation": distribution(local_validation_ms),
        "application_ack_rtt": distribution(acknowledgement_ms),
        "transport_round_trip_excluding_server_residence": distribution(transport_rtt_ms),
        "server_feedback_wait": distribution(response_wait_ms),
        "acknowledgement_median_ms": sorted(acknowledgement_ms)[len(acknowledgement_ms)//2] if acknowledgement_ms else None,
        "note": "真实厂家双臂 IK + SSH/JSON + 机器人 FK；transport RTT 用两端各自时钟的时长相减，无单程时钟假设。只读 ROS 模式订阅硬件反馈；不含运动话题发布、控制器采纳和电机响应，不是物理端到端延迟。"}
    path = connection.log_path.with_suffix(".link-check.json")
    path.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print("通信测试结果：", path, flush=True)
    accepted = remote.get("accepted", {}).get("samples", 0)
    complete = accepted == len(sent_times) and not remote.get("errors")
    print(f"下发 {summary['sent']['hz']:.1f} Hz / 有效接收 {remote.get('accepted', {}).get('hz', 0.):.1f} Hz；"
          + ("全部目标收到。" if complete else "有目标未被逐帧处理，当前链路尚未通过完整接收检查。"), flush=True)
    return 0 if complete else 1
