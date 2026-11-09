# Fairino holonomic lookahead controller

This is an actual ROS 2 Humble `nav2_core::Controller` plugin. Nav2 retains the
planner, `FollowPath` action, lifecycle, progress checker, cancellation and
replanning. The plugin replaces DWB's local trajectory sampler with a holonomic
lookahead controller; existing `velocity_gate` and chassis guards remain after it.

## Control and frame contract

- The lookahead distance is a Euclidean radius around the current robot position,
  not path arc length. Path progress determines the first forward circle/path
  intersection. A terminal segment inside the circle may use its endpoint;
  remaining paths that fold back are rejected to avoid shortcutting a loop.
- `heading_mode: hold_initial` remains the default: the request records current
  `map → base_footprint` yaw and the plugin holds that heading throughout the path.
- `heading_mode: path_tangent` points along the forward chord from the path
  projection to the selected lookahead point. Lateral tracking error does not
  itself become a heading target. A degenerate chord uses the next valid forward
  segment, or the terminal yaw when no forward segment remains.
- On the terminal tail, as distance to the goal decreases from
  `lookahead_distance` to `xy_goal_tolerance`, `path_tangent` blends along the
  shortest angular direction to the requested terminal yaw. Inside the position
  tolerance it uses that exact terminal yaw. A single-point path directly uses
  terminal yaw, including rotation in place. Translation can continue while yaw
  changes; there is no turn-first phase and no new tuning parameter.
- SmacPlanner2D must keep `use_final_approach_orientation: false` so replanning
  preserves the requested terminal quaternion. `setPlan()` does not recapture
  current heading or clear the velocity history in either mode.
- Pose errors are evaluated in the path's frame. The command is converted to the
  body frame and contains `vx`, `vy`, and `wz`. Linear magnitude and angular speed
  are limited, followed by linear-vector and angular acceleration limits.
- A `StoppedGoalChecker` confirms position, heading and measured stopped velocity.
  Inside the goal tolerances, the controller ramps the target velocity to zero.
- Fresh pose, usable TF, current costmap and a valid forward path are required.
  Failure raises `nav2_core::PlannerException`; ControllerServer publishes zero.
  Protective stops bypass the normal acceleration ramp.

## Chassis motion coordination

The plugin requires fresh diagnostic_msgs/DiagnosticArray feedback on
/fairino_navigation/chassis_motion_state (BEST_EFFORT, VOLATILE, depth 1).
The worker reports observed drive commands, not the vendor's private state-machine
phase: drive_wait means a meaningful requested Twist with all drive commands
near zero. It can include alignment or braking, and is not an ALIGN_ONLY claim.

- A ready idle observation permits the first normal request. During drive_wait,
  path progress still uses actual pose, but the follower does not advance its
  velocity ramp. The body-frame vx:vy:wz ratio is held using the worker's actual
  request when consistent with the current path, and scaled down together to
  coordination_linear_speed (0.02 m/s) and coordination_angular_speed (0.04 rad/s).
- A significant change of the un-ramped path request rebuilds the small intent;
  repeated paths and small changes do not chase the wheel direction.
  coordination_direction_tolerance defaults to 0.20 rad, measured between
  vectors normalized by the waiting linear and angular limits. A near-zero
  component cannot scale another useful component into the no-request deadband.
- Reaching a zero target clears the held intent immediately. Ending a wait
  reseeds the normal acceleration ramp from current measured velocity.
  Final limited output and measured motion both retain collision checks.
- chassis_feedback_timeout defaults to 0.4 s and is generated from the chassis
  feedback limit. Source age, local receipt age and total hardware-sample age
  must all remain within it, including after TF/collision processing.
- One continuous wait is bounded by coordination_wait_timeout (15 s).
  Replans, rebuilt intent and protective-stop retries do not restart that budget.
  Receiving an actual non-wait observation ends the episode. Existing Nav2
  progress, TF, collision and cancellation protections remain in force.
- A status must have the exact schema, consistent state fields, a strictly
  increasing sequence and one lowercase 32-hex worker session. A worker-session
  change is rejected until navigation is restarted; the plugin never silently
  adopts a newly connected chassis during an active navigation session.
  Stale, malformed, unhealthy or unready feedback raises PlannerException.

Waiting caps must exceed the 1e-4 motion-request threshold and be no larger than
navigation speed limits. An intent that still falls into that deadband after
vector scaling is rejected rather than repeatedly exiting alignment.
The direction tolerance must be strictly between zero and pi/2; the wait timeout
must exceed the feedback timeout and be no larger than 60 s. These controls coordinate requests with the
observed drive response; they do not modify motor current limits or establish
the cause of a hardware drive fault.

## Collision policy

The actual limited command and current measured motion are projected in the local
costmap. The horizon includes at least the configured time and stopping time plus
one control period. Padded footprint edges **and interior** are checked against
occupied and unknown cells. Leaving the local costmap is rejected. Sampling is
bounded by both the configured time step and half a costmap cell of maximum
footprint-point motion, including rotation. More than 4,000 projection steps is
rejected rather than allowing unbounded work in the control cycle.

This controller does not search alternative local velocity trajectories like
DWB. A blocked direct following command stops; the existing global planner may
produce a new path, or Nav2 reports failure according to its configured timeout.

## Build and verification

Build against the robot's ROS 2 Humble/Nav2 installation, for example in an
isolated workstation workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select fairino_lookahead_controller
source install/setup.bash
colcon test --packages-select fairino_lookahead_controller
colcon test-result --verbose
ros2 run fairino_lookahead_controller plugin_check
```

`plugin_check` only loads, constructs and destroys the plugin through pluginlib.
It creates no ROS node and sends no commands. `test_plugin` uses a local empty
costmap and no robot interfaces to test lifecycle, body-frame output, pose
rejection, collision, external speed-limit handling and chassis wait/release. Run ROS tests in an
isolated ROS domain with `ROS_LOCALHOST_ONLY=1`.

The same numerical, collision and coordination headers used at runtime are also compiled by
the repository's `scripts/check_controller.sh`, without ROS. This validates core
behavior on a development computer, including a stopped-wheel alignment / drive /
goal-stop closed loop, malformed feedback, replay, stale sample rejection,
direction changes, pure rotation and measured-speed restart. It does not prove
ROS plugin loading or physical motion. Full Humble compilation and tests require a Humble environment.

`package.xml` has placeholder maintainer contact and unspecified licensing; fill
these with the project owner's approved metadata before publishing externally.
