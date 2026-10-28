"""The complete runtime bundle must survive a bounded SSH argument."""
from pathlib import Path
import shlex
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools'))
from install_navigation import remote_command, remote_script


class InstallationTransportTests(unittest.TestCase):
    def test_both_modes_deliver_the_complete_program_without_large_ssh_argument(self):
        for execute in (False, True):
            with self.subTest(execute=execute):
                argv = shlex.split(remote_command(execute))
                self.assertEqual(argv[:3], ['python3', '-u', '-c'])
                self.assertLess(len(argv[3].encode()), 65536)
                captured = []
                # Decode the real transport loader, but never execute an installer.
                with patch('builtins.compile', side_effect=lambda source, *args: captured.append(source) or 'pass'):
                    exec(argv[3], {'exec': lambda *args: None})
                self.assertEqual(captured, [remote_script(execute).encode()])


if __name__ == '__main__':
    unittest.main()
