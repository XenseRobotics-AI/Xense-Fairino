import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import setup_dxhand


class RuntimeTests(unittest.TestCase):
    def test_corruption_stops_setup_before_build_or_sdk_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "upstream/vendor/manussdk"
            source.mkdir(parents=True)
            payload = source / "lib.so.1.0"
            link = source / "lib.so.1"
            payload.write_bytes(b"licensed test fixture")
            link.symlink_to(payload.name)

            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({
                payload.name: {"sha256": hashlib.sha256(payload.read_bytes()).hexdigest()},
                link.name: {"symlink": payload.name, "sha256": hashlib.sha256(payload.name.encode()).hexdigest()},
            }))
            local = root / "local"
            installed = local / "vendor/manussdk/lib.so.1.0"
            installed.parent.mkdir(parents=True)
            installed.write_bytes(b"previous installation")

            with patch.object(setup_dxhand, "LOCAL", local), patch.object(setup_dxhand, "MANIFEST", manifest), \
                    patch("setup_dxhand.subprocess.run") as build:
                link.unlink()
                link.symlink_to("unverified.so")
                with self.assertRaisesRegex(ValueError, "symlink mismatch"):
                    setup_dxhand.setup(root / "upstream")
                link.unlink()
                link.symlink_to(payload.name)
                payload.write_bytes(b"corrupt")
                with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                    setup_dxhand.setup(root / "upstream")
                build.assert_not_called()
                self.assertEqual(installed.read_bytes(), b"previous installation")
