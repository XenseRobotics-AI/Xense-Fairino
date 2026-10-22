"""Exercise the real startup sequence against fake ROS endpoints, without motors."""
import copy
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_worker as rw
from control_core import STEER, normalise_hardware, chassis_parameters
from test_control import hardware


CONFIG = {
    "angle_controller": {"ros__parameters": {"joints": ["j1", "j3", "j5", "j7"]}},
    "eight_dof_chassis_controller": {"ros__parameters": {
        "limits": {"base": {"linear": {}, "angular": {}}, "steering": {}, "drive": {}},
        "modules": {name: {"wheel_radius": .065, "base_to_steering": {"xyz": xyz}}
                    for name, xyz in zip(("front_left", "front_right", "rear_left", "rear_right"),
                        ((.31, .225, 0), (.31, -.225, 0), (-.305, .2175, 0), (-.305, -.2175, 0)))}}},
}

CONFIG["eight_dof_chassis_controller"]["ros__parameters"] = chassis_parameters(CONFIG["eight_dof_chassis_controller"]["ros__parameters"])

def goal():
    return NS(trajectory=NS(joint_names=[], points=[]), path_tolerance=[],
              goal_tolerance=[], goal_time_tolerance=NS(sec=0))


class StartupTests(unittest.TestCase):
    def exercise(self, trajectory_error=0, prepare=False, initially_enabled=False,
                 wrong_topic=False, existing_chassis=False, stale=None, check=False,
                 unload_ok=True):
        worker = rw.Worker.__new__(rw.Worker)
        worker.args = NS(check=check, prepare_check=prepare)
        worker.hw = normalise_hardware(hardware())
        for i, p in zip(STEER, [.2, -.2, .72, -.35]):
            worker.hw["position"][i] = p
            worker.hw["command_position"][i] = p
        worker.hw["enabled"][0] = initially_enabled
        worker.hw_time = worker.upper_time = time.monotonic()
        worker.stop = threading.Event()
        worker.pub = None
        worker.ready = worker.owned = worker.enable_attempted = False
        worker.loaded = []
        worker.loaded_params = {}
        worker.commands = []
        worker.trace = []
        worker.goals = []
        worker.guard = lambda *a: []
        worker.wait_initial_feedback = lambda: None
        def pump(*args):
            worker.hw_time = worker.upper_time = time.monotonic()
        worker.pump = pump
        worker.status = lambda *a, **k: None
        controls = {"fairino_hardware_command_controller": "active",
                    **({rw.CHASSIS: "inactive"} if existing_chassis else {}), **(stale or {})}
        worker.controllers = lambda *a: dict(controls)
        def service(typ, path, request):
            self.assertEqual(path, '/controller_manager/unload_controller')
            worker.trace.append(('unload', request.name))
            if unload_ok:
                del controls[request.name]
            return NS(ok=unload_ok)
        worker.service = service
        worker.emit = lambda **kw: None
        worker.odometry = lambda action, **k: worker.trace.append(('odometry', action))
        worker.verify_chassis = lambda params: worker.trace.append(('verify', params))
        def load(name, typ, params):
            worker.loaded.append(name)
            worker.loaded_params[name] = copy.deepcopy(params)
        worker.load = load
        worker.switch = lambda activate, deactivate: worker.trace.append(("switch", activate, deactivate))
        worker.future = lambda result, *a: result
        worker.node = NS(create_publisher=lambda *a: NS(publish=lambda *a: None),
                         get_subscriptions_info_by_topic=lambda topic: (
                             [NS(node_name=rw.CHASSIS)] if topic == rw.TOPIC or wrong_topic else []))

        def command(text):
            worker.commands.append(text)
            worker.trace.append(("command", text))
            if text.startswith("enable"):
                mask = 85 if text == "enable(85)" else 255
                worker.hw["enabled"] = [bool(mask & (1 << i)) for i in range(8)]
            elif text.startswith("disable"):
                worker.hw["enabled"] = [False]*8
        worker.command = command

        def result():
            if not trajectory_error:
                for i in STEER:
                    worker.hw["position"][i] = worker.hw["command_position"][i] = 0.0
            return NS(status=4 if not trajectory_error else 6,
                      result=NS(error_code=trajectory_error, error_string="simulated tracking fault"))

        def send(trajectory):
            self.assertEqual(worker.hw["enabled"], [True, False]*4)
            self.assertEqual(trajectory.trajectory.joint_names, ["j1", "j3", "j5", "j7"])
            self.assertEqual(trajectory.trajectory.points[-1].positions, [0.0]*4)
            worker.goals.append(trajectory)
            return NS(accepted=True, get_result_async=result)
        worker.follow = NS(server_is_ready=lambda: True, send_goal_async=send)
        globals_for_ros = {
            "yaml": NS(safe_load=lambda _: copy.deepcopy(CONFIG)),
            "Path": lambda _: NS(read_text=lambda: ""),
            "FollowJointTrajectory": NS(Goal=goal),
            "JointTrajectoryPoint": NS,
            "JointTolerance": NS,
            "Duration": lambda seconds: NS(to_msg=lambda: seconds),
            "Twist": NS,
            "UnloadController": NS(Request=NS),
        }
        with patch.dict(rw.__dict__, globals_for_ros):
            try:
                worker.startup()
            except RuntimeError as exc:
                worker.failure = str(exc)
            else:
                worker.failure = None
        return worker

    def test_homing_finishes_before_drive_enable(self):
        w = self.exercise()
        self.assertIsNone(w.failure)
        self.assertTrue(w.ready)
        self.assertEqual(w.commands[-3:], ["enable(85)", "disable(85)", "enable()"])
        self.assertEqual(len(w.goals), 1)

    def test_existing_factory_instance_is_reused_and_odom_source_is_stable(self):
        w = self.exercise(existing_chassis=True)
        self.assertIsNone(w.failure)
        self.assertNotIn(rw.CHASSIS, w.loaded)
        self.assertEqual(rw.CHASSIS, 'eight_dof_chassis_controller')
        self.assertEqual(rw.ODOM, '/eight_dof_chassis_controller/odom')
        self.assertEqual([x[1] for x in w.trace if x[0] == 'odometry'], ['pause', 'resume'])
        self.assertTrue(any(x[0] == 'verify' for x in w.trace))

    def test_failed_homing_never_enables_drive_motors(self):
        w = self.exercise(trajectory_error=-4)
        self.assertIn("回正失败", w.failure)
        self.assertNotIn("enable()", w.commands)
        self.assertFalse(w.ready)

    def test_prepare_check_never_enables_or_sends_trajectory(self):
        w = self.exercise(prepare=True)
        self.assertIsNone(w.failure)
        self.assertFalse(any(c.startswith("enable") for c in w.commands))
        self.assertFalse(w.goals)

    def test_enabled_robot_is_not_taken_over(self):
        w = self.exercise(initially_enabled=True)
        self.assertIn("拒绝接管", w.failure)
        self.assertFalse(w.commands)

    def test_disabled_inactive_residue_is_recovered_before_homing(self):
        w = self.exercise(existing_chassis=True, stale={rw.ANGLE: 'inactive'})
        self.assertIsNone(w.failure)
        self.assertTrue(w.ready)
        self.assertEqual([x for x in w.trace if x[0] == 'unload'], [('unload', rw.ANGLE)])
        self.assertLess(w.trace.index(('unload', rw.ANGLE)),
                        w.trace.index(('command', 'enable(85)')))
        self.assertNotIn(rw.CHASSIS, w.loaded)

    def test_failed_recovery_blocks_all_startup_commands(self):
        w = self.exercise(stale={rw.ANGLE: 'inactive'}, unload_ok=False)
        self.assertIn('回收上次临时控制器失败', w.failure)
        self.assertFalse(w.commands)
        self.assertFalse(w.owned)
        self.assertFalse(w.goals)

    def test_readonly_check_never_recovers_residue(self):
        w = self.exercise(check=True, stale={rw.ANGLE: 'inactive'})
        self.assertIsNone(w.failure)
        self.assertFalse(w.trace)
        self.assertFalse(w.loaded)
        self.assertFalse(w.owned)

    def test_web_topic_leak_blocks_drive_enable(self):
        w = self.exercise(wrong_topic=True)
        self.assertIn("网页速度话题", w.failure)
        self.assertNotIn("enable()", w.commands)


if __name__ == "__main__":
    unittest.main()
