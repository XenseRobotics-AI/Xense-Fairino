"""Selected TCP survives PICO planning, Home and independent remote checking."""
import unittest
import ast
import base64
import json
import io
import shlex
import sys
import types
import zlib
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import paths
from tcp_core import ARMS, target_matrix
from tool_frames import load_geometry, tool_config
from vendor_kinematics import VendorKinematics
from vr_plan import request
from vr_core import validate
from vr_home import load_home, HomeMotion
from test_tcp import hardware
from vr_client import remote_command, main


class VRToolTests(unittest.TestCase):
    def setUp(self):
        geometry = load_geometry()
        geometry['arms']['left'].update(roll_deg=90., confirmed=True)
        geometry['arms']['right'].update(roll_deg=-90., confirmed=True)
        self.tools = tool_config('taccap', geometry, True)
        self.kin = VendorKinematics(tools=self.tools)
        self.hw = hardware()

    def test_tip_goals_are_verified_with_the_same_selected_tool(self):
        goals = {s: target_matrix(self.kin.forward_tcp(
            [self.hw['command_position'][j-1] for j in js], s)[0], 3, 1, .1)
            for s, js in ARMS.items()}
        packet = request(self.kin, self.hw, goals, 1, 10.)
        self.assertEqual(set(validate(self.hw, packet, 10., self.tools)), set(ARMS))
        with self.assertRaisesRegex(ValueError, '工具配置不一致'):
            validate(self.hw, packet, 10.)
        packet['arms']['left']['tcp'][3] += 10.
        with self.assertRaisesRegex(ValueError, '正解校验'):
            validate(self.hw, packet, 10., self.tools)

    def test_home_joint_targets_stay_the_same_with_a_different_tcp(self):
        home = HomeMotion(load_home(), self.hw, 10.)
        packet = home.request(self.kin, self.hw, 10.01, 1, 100.)
        self.assertEqual(packet['tool_id'], self.tools['id'])
        self.assertEqual(set(validate(self.hw, packet, 100., self.tools)), set(ARMS))
        for side, values in packet['arms'].items():
            np.testing.assert_allclose(values['tcp'], np.array(
                self.kin.forward_tcp(values['target'], side)[0]).flatten(), atol=1e-9)

    def test_remote_bundle_accepts_tools_without_local_paths_or_cad_packages(self):
        command = remote_command(SimpleNamespace(execute=True, prepare_check=False, tools=self.tools))
        setup = shlex.split(command)[2]
        source = shlex.split(setup.split(' -u -c ', 1)[1])[0]
        tree = ast.parse(source)
        encoded = next(n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and ast.unparse(n.func) == 'base64.b64decode')
        bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
        modules = {}
        with patch.dict(sys.modules):
            for name in ('tcp_core', 'tool_frames'):
                module = types.ModuleType(name)
                sys.modules[name] = modules[name] = module
                exec(compile(bundle[name], name+'.py', 'exec'), module.__dict__)
            remote = modules['tool_frames']
            self.assertIsNone(remote.DEFAULT_CONFIG)
            self.assertEqual(remote.validate_runtime(self.tools), self.tools)
            self.assertNotIn('numpy', bundle['tool_frames'])

    def test_unconfirmed_cli_stops_before_connecting(self):
        data = load_geometry()
        for spec in data['arms'].values():
            spec['confirmed'] = False
        with patch('vr_client.load_geometry', return_value=data), patch('vr_client.Connection') as connection, \
                patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as result:
                main(['--execute', '--tcp-tool', 'taccap'])
            self.assertEqual(result.exception.code, 2)
            connection.assert_not_called()


if __name__ == '__main__':
    unittest.main()
