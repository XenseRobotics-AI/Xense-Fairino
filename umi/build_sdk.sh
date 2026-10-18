#!/usr/bin/env bash
set -euo pipefail

sdk_root="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
sdk_source="$sdk_root/vendor/TacCap-Gripper"
sdk_python="$sdk_root/.venv/bin/python"

if [[ ! -x "$sdk_python" ]]; then
    python3 -m venv --system-site-packages "$sdk_root/.venv"
fi

(
    cd "$sdk_root/wheels"
    sha256sum -c SHA256SUMS
)
"$sdk_python" -m pip --disable-pip-version-check install --no-index \
    --find-links "$sdk_root/wheels" pip==25.3 packaging==24.2 \
    scikit-build-core==0.11.6 pybind11==2.13.6 pyserial==3.5

mkdir -p "$sdk_root/dist" "$sdk_root/build-logs"
cd "$sdk_source"
nice -n 10 env CMAKE_GENERATOR='Unix Makefiles' \
    CMAKE_BUILD_PARALLEL_LEVEL="${SDK_BUILD_JOBS:-2}" \
    "$sdk_python" -m pip --disable-pip-version-check wheel . \
    --no-deps --no-build-isolation --no-index --wheel-dir "$sdk_root/dist" \
    --config-settings=cmake.define.TACCAP_BUILD_TESTS=ON \
    --config-settings=cmake.define.TACCAP_BUILD_EXAMPLES=ON \
    --config-settings=cmake.define.pybind11_DIR="$("$sdk_python" -m pybind11 --cmakedir)" \
    --verbose 2>&1 | tee "$sdk_root/build-logs/build.log"

sdk_version="$("$sdk_python" -c 'import pathlib,sys,tomli; print(tomli.loads(pathlib.Path(sys.argv[1]).read_text())["project"]["version"])' "$sdk_source/pyproject.toml")"
"$sdk_python" -m pip --disable-pip-version-check install --no-index --no-deps \
    --force-reinstall --find-links "$sdk_root/dist" "taccap-gripper==$sdk_version"
cd "$sdk_root"
env -u PYTHONPATH "$sdk_python" -c 'import xense.taccap as t; print(t.hello()); print(t.__file__)'
