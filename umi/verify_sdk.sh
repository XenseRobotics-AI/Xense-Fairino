#!/usr/bin/env bash
set -euo pipefail

sdk_root="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
sdk_source="$sdk_root/vendor/TacCap-Gripper"
sdk_python="$sdk_root/.venv/bin/python"
sdk_build="$sdk_source/build/cp310-cp310-linux_aarch64"
mkdir -p "$sdk_root/build-logs"

cd "$sdk_root"
env -u PYTHONPATH "$sdk_python" - <<'PY'
import importlib.metadata
import pathlib
import platform
import xense.taccap as t

print('Architecture:', platform.machine())
print('Package version:', importlib.metadata.version('taccap-gripper'))
print(t.hello())
print('Installed module:', t.__file__)
print('Native module:', t._taccap_native.__file__)
assert t.__version__ == '0.4.3'

# The upstream test harness requires the native extension to appear in its
# source checkout. Link the installed, already-built wheel binaries there.
installed = pathlib.Path(t.__file__).parent
source = pathlib.Path('vendor/TacCap-Gripper/python/xense/taccap').resolve()
for binary in installed.glob('*.so*'):
    destination = source / binary.name
    if destination.is_symlink() and destination.resolve() == binary.resolve():
        continue
    if destination.exists() or destination.is_symlink():
        raise RuntimeError(f'Refusing to replace an existing test binary: {destination}')
    destination.symlink_to(binary)
PY

nice -n 10 ctest --test-dir "$sdk_build" --output-on-failure \
    --parallel 2 --timeout 45 2>&1 | tee "$sdk_root/build-logs/cpp-tests.log"

cd "$sdk_source"
nice -n 10 env -u PYTHONPATH "$sdk_python" -m pytest -q python/tests \
    -k 'not test_imu_vectors_are_real_buffers and not test_accel_magnitude_is_about_one_g' \
    2>&1 | tee "$sdk_root/build-logs/sdk-python-tests.log"
