#!/usr/bin/env bash
# Ubuntu 24.04 x86_64. Installs only inside this bundle; no robot service changes.
set -euo pipefail
PICO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PICO_RUNTIME="$PICO_ROOT/.local/pico4"
source /etc/os-release
if [[ "$(uname -m)" != x86_64 || "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo '此功能包的 PICO 服务支持 Ubuntu 24.04 x86_64。' >&2; exit 1
fi
for name in cmake g++ dpkg-deb unzip curl; do
  if ! command -v "$name" >/dev/null; then
    echo '请安装：sudo apt install python3-venv python3-dev build-essential cmake unzip curl libusb-1.0-0' >&2; exit 1
  fi
done
mkdir -p "$PICO_RUNTIME/downloads"
for asset in "$PICO_ROOT"/vendor/assets/*; do
  [[ -f "$asset" ]] || continue
  [[ -f "$PICO_RUNTIME/downloads/$(basename "$asset")" ]] || cp "$asset" "$PICO_RUNTIME/downloads/"
done
if [[ ! -x "$PICO_RUNTIME/env/bin/python" ]]; then
  if [[ "${PICO_USE_CONDA:-0}" == 1 ]]; then
    PICO_CONDA=$(command -v conda || true)
    [[ -n "$PICO_CONDA" ]] || PICO_CONDA="$HOME/miniconda3/bin/conda"
    "$PICO_CONDA" create --prefix "$PICO_RUNTIME/env" python=3.12 pip -y
  else
    if ! /usr/bin/python3 -c 'import ensurepip' >/dev/null 2>&1 || [[ ! -f /usr/include/python3.12/Python.h ]]; then
      echo '请先安装 python3-venv python3-dev；已有 Conda 可运行 PICO_USE_CONDA=1 bash setup.sh pico。' >&2; exit 1
    fi
    /usr/bin/python3 -m venv "$PICO_RUNTIME/env"
  fi
fi
"$PICO_RUNTIME/env/bin/python" -m pip install -r "$PICO_ROOT/scripts/pico4/requirements.txt"
"$PICO_RUNTIME/env/bin/python" "$PICO_ROOT/scripts/pico4/fetch_assets.py"
dpkg-deb -x "$PICO_RUNTIME/downloads/XRoboToolkit_PC_Service_1.0.0_ubuntu_24.04_amd64.deb" "$PICO_RUNTIME/service"
if [[ ! -x "$PICO_RUNTIME/platform-tools/adb" ]]; then
  if [[ ! -f "$PICO_RUNTIME/downloads/platform-tools-latest-linux.zip" ]]; then
    curl -fLsS --connect-timeout 20 --max-time 300 --retry 3 \
      https://dl.google.com/android/repository/platform-tools-latest-linux.zip \
      -o "$PICO_RUNTIME/downloads/platform-tools-latest-linux.zip.part"
    mv "$PICO_RUNTIME/downloads/platform-tools-latest-linux.zip.part" \
      "$PICO_RUNTIME/downloads/platform-tools-latest-linux.zip"
  fi
  unzip -q -o "$PICO_RUNTIME/downloads/platform-tools-latest-linux.zip" -d "$PICO_RUNTIME"
fi
PICO_BINDING="$PICO_ROOT/src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind"
PICO_SITE=$("$PICO_RUNTIME/env/bin/python" -c 'import sysconfig; print(sysconfig.get_path("platlib"))')
PICO_PYBIND_CMAKE=$("$PICO_RUNTIME/env/bin/python" -m pybind11 --cmakedir)
cmake -S "$PICO_BINDING" -B "$PICO_RUNTIME/binding-build" \
  -DPYTHON_EXECUTABLE="$PICO_RUNTIME/env/bin/python" \
  -Dpybind11_DIR="$PICO_PYBIND_CMAKE" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$PICO_SITE"
cmake --build "$PICO_RUNTIME/binding-build" -j2
"$PICO_RUNTIME/env/bin/python" -m pip freeze > "$PICO_RUNTIME/requirements-installed.txt"
bash "$PICO_ROOT/scripts/pico4/run.sh" check
