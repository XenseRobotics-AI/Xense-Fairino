"""Build DXHand offline using the licensed Manus files from TsenseIV2C-main."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "third_party/tsensei_retarget"
LOCAL = ROOT / ".local/dxhand"
UPSTREAM = Path.home() / "Olorin/05_DXHand/TsenseIV2C-main"
MANIFEST = Path(__file__).with_name("manus-runtime.json")


def copy_runtime(source, stage):
    """Verify the staged bytes. Link entries hash the literal link target."""
    entries = json.loads(MANIFEST.read_text())
    for name, entry in entries.items():
        src, dst = source / name, stage / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        if "symlink" in entry:
            if not src.is_symlink() or os.readlink(src) != entry["symlink"]:
                raise ValueError(f"Manus symlink mismatch: {src}")
            dst.symlink_to(entry["symlink"])
            digest = hashlib.sha256(os.fsencode(os.readlink(dst))).hexdigest()
        else:
            if src.is_symlink() or not src.is_file():
                raise ValueError(f"Expected a regular Manus SDK file: {src}")
            shutil.copyfile(src, dst)
            with dst.open("rb") as file:
                digest = hashlib.file_digest(file, "sha256").hexdigest()
        if digest != entry["sha256"]:
            raise ValueError(f"Manus SHA-256 mismatch: {src}")
    print(f"Verified {len(entries)} Manus SDK files and symlinks from {source}", flush=True)


def setup(upstream):
    if sys.version_info[:2] != (3, 12):
        raise SystemExit("Run bash setup.sh dxhand with Python 3.12 installed")
    tmp = LOCAL / "tmp"
    tmp.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TMPDIR=str(tmp), PIP_CACHE_DIR=str(LOCAL / "pip-cache"),
               PYTHONDONTWRITEBYTECODE="1", PIP_DISABLE_PIP_VERSION_CHECK="1")

    def run(*args):
        subprocess.run(args, cwd=LOCAL, env=env, check=True)

    python = LOCAL / "env/bin/python"
    with (tmp / "setup.lock").open("a") as lock, tempfile.TemporaryDirectory(dir=tmp) as directory:
        fcntl.flock(lock, fcntl.LOCK_EX)
        work = Path(directory)
        sdk = work / "manussdk"
        shutil.copytree(SOURCE / "vendor/manussdk", sdk)
        copy_runtime(upstream / "vendor/manussdk", sdk)
        run(sys.executable, "-m", "venv", LOCAL / "env")
        run(python, "-m", "pip", "install", "numpy==2.4.4", "scipy==1.17.1",
            "daqp==0.8.7", "PyYAML==6.0.3", "ruckig==0.19.4", "pybind11==2.13.6")

        # A fresh build also replaces any SDK objects from the previous snapshot.
        build = work / "build"
        run("cmake", "-S", ROOT / "tools/dxhand", "-B", build, "-DCMAKE_BUILD_TYPE=Release",
            f"-DPython3_EXECUTABLE={python}", f"-DA32_NATIVE_OUTPUT={work}")
        run("cmake", "--build", build, "--parallel", "2")
        run("ctest", "--test-dir", build, "--output-on-failure")

        script = LOCAL / "scripts/build_manus.sh"
        script.parent.mkdir(exist_ok=True)
        shutil.copyfile(SOURCE / "scripts/build_manus.sh", script)
        # Build at its final path so the SDK-relative RPATH and CMake cache agree.
        installed = LOCAL / "vendor/manussdk"
        installed.parent.mkdir(exist_ok=True)
        previous = work / "previous-sdk"
        if installed.exists():
            installed.rename(previous)
        try:
            sdk.rename(installed)
            run("bash", script, python)
        except BaseException:
            if installed.exists():
                installed.rename(work / "failed-sdk")
            if previous.exists():
                previous.rename(installed)
            raise
        (LOCAL / "lib").mkdir(exist_ok=True)
        (work / "liba32_native.so").replace(LOCAL / "lib/liba32_native.so")
    print("DXHand Python 3.12 / Manus 3.1.1 / A3 SDK 1.1.15 offline setup complete")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, default=UPSTREAM,
                        help="TsenseIV2C-main source directory (read-only)")
    setup(parser.parse_args().upstream.expanduser().resolve())
