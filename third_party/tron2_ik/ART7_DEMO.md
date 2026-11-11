# ART7 PICO integration (robot and Demo)

This directory contains the MIT-licensed C++ solver and Python facade from
`/home/lim/Downloads/tron2-ik-main.zip` (copyright 2026 dongyaolin). The original
TRON2 URDF, meshes, Qt and MeshCat applications are deliberately omitted. The
original README is retained as upstream documentation, not the ART7 setup guide.
Source archive SHA-256:
`37a50c02979d680de8d5d58c17dda1acad9926a87cebe9ccf4b2ce6a630bdfe0`.

The only upstream build-source modification is `TRON2_EIGEN_INCLUDE` support
in `setup.py`. The original C++ solver is unchanged.

The repository adapter is `tools/ik_experiments/tron2_art7.py`. It generates a
14-axis ART7 model from the same DH constants, joint limits and runtime TCP
configuration as the independent model. Upstream TRON2 joint names in that
generated file are aliases, not TRON2 geometry. Each arm's targets remain in
its own arm-base coordinates, as required by the existing PICO input mapping.

The upstream TRON2 center-column avoidance approximation is disabled because
its geometry does not describe ART7. The solver uses individual joint bounds;
the Demo packet adapter separately enforces the optional ART7 wrist envelope
and independently validates FK and error reduction. The real PICO adapter is
`scripts/pico4/fairino/vr_tron_ik.py`; it requires the wrist guard, checks measured
feedback and velocity reserve, and emits the existing receiver protocol. The
receiver repeats path and guard checks. Demo subclasses the same adapter but
may disable the guard; its packets remain marked and rejected by the robot.
This port does not add whole-robot collision detection.

From the repository root, build with the PICO interpreter:

```bash
.local/pico4/env/bin/python tools/ik_experiments/setup_tron2_ik.py --install
```

This downloads optional dependencies into `.local/tron2-ik`, without changing
the PICO environment. Critical native dependencies are pinned: Pinocchio 3.9.0,
eiquadprog 1.3.0, urdfdom 4.0.1, tinyxml2 10.0.0 and Eigen 3.4.1. The extension
is built into a source/ABI-keyed ignored directory. Repeat without `--install`
to rebuild offline. PICO startup performs neither downloads nor compilation.
An unavailable extension produces an explicit setup error, never a fallback
to another IK implementation.

```bash
.local/pico4/env/bin/python -m unittest tools.ik_experiments.test_tron2_art7 -v
```

Select `TRON2 QP IK` in the console's real-robot target or run
`bash run.sh vr --ik-backend tron2`. The default remains the existing native IK.
Both real native and TRON2 branches now request 500 Hz client / 500 Hz receiver
scheduling. They retain the original vendor chain's Cartesian position-linear /
orientation-SLERP interpolation before IK, with 12 ms buffering and direct joint
publication. There is no added receiver-side joint interpolation. Overdue slots
are skipped, and actual rates remain observable. Model loading and validation
happen before connecting devices. No live commissioning is implied by the
offline validation.

The ART7 wrapper resolves the original native module once at construction,
then calls the same C++ model directly with the upstream option conversion.
It retains per-call option/pose/joint validation while avoiding repeated library
discovery in the 500 Hz loop. Equivalence tests compare results with the original
Python facade; the vendored C++ and Python sources remain unchanged.

The core runs the original dual-arm QP in C++. Its internal `dt`, step norm and
generated URDF velocity entries bound numerical iteration steps only. They
are not an actuator trajectory/velocity limiter. The application currently
caps solves at 120 iterations, with 0.02 mm position and 0.01 degree orientation
tolerances; partial or unreachable targets require the packet adapter's checks.
