# kuka_control_tutorial

A worked example of driving the KUKA LBR end effector in Cartesian space through
the Cartesian impedance controller — from a single `PoseStamped` up to a
mouse-drawn trajectory with live stiffness tuning.

Runs on Gazebo and on the real arm.

## The three things this teaches

| Guide | What it covers |
|---|---|
| **[Commanding an end-effector pose](doc/commanding-ee-pose.md)** | The one topic that moves the robot: its name, type, the frame rule that silently eats bad messages, why a distant setpoint is a *force* and not a destination, and every module that touches it. |
| **[Composing an action and a trajectory](doc/actions-and-trajectories.md)** | Anatomy of a `.action` file, the three server callbacks, the sampler abstraction that lets both actions share one streaming loop, and a recipe for adding a third. |
| **[Dynamically configurable stiffness](doc/dynamic-stiffness.md)** | The three entry points — GUI panel, `ros2 param`, Python client — and the path a value takes through validation, atomics and rate limiting into the control law. |

Start with the first one. The rest of the package only makes sense once the
spring analogy has landed.

Already know what you want to do? **[Build your own
controller](#build-your-own-controller)** indexes the jobs you will hit against
the exact lines here that solve them.

## The idea in one paragraph

`cartesian_impedance_controller` takes a **single** setpoint pose on
`target_frame` and pulls the end effector towards it with a spring. Publishing
the final goal in one message stretches that spring by the whole travel
distance — a jolt, and a large force into whatever the arm is touching. So
everything here streams closely spaced setpoints at 100 Hz that walk to the
goal, keeping the spring short. Actions wrap that streaming so motions can be
watched and cancelled.

## Layout

| Path | What it is |
|---|---|
| [action/MoveToPose.action](action/MoveToPose.action) | Straight line to a point; the server picks the timing |
| [action/FollowPath.action](action/FollowPath.action) | Arbitrary path; the *caller* supplies the timing |
| [cartesian_move_server.py](kuka_control_tutorial/cartesian_move_server.py) | Both action servers, sharing one streaming core. **The only code that publishes setpoints.** |
| [motion_client.py](kuka_control_tutorial/motion_client.py) | Blocking client wrapper for both actions |
| [stiffness_client.py](kuka_control_tutorial/stiffness_client.py) | Reads and writes the controller's stiffness parameters |
| [draw_trajectory_gui.py](kuka_control_tutorial/draw_trajectory_gui.py) | The mouse-drawing demo and stiffness panel |
| [trajectory.py](kuka_control_tutorial/trajectory.py) | Velocity profile and quaternion maths — ROS-free |
| [path_planning.py](kuka_control_tutorial/path_planning.py) | Drawn-stroke conditioning — ROS-free |
| [move_to_pose_client.py](kuka_control_tutorial/move_to_pose_client.py) | Command line client for single moves |
| [config/move_to_pose.yaml](config/move_to_pose.yaml) | Every parameter, documented inline |
| [launch/move_to_pose.launch.py](launch/move_to_pose.launch.py) | Starts the server, optionally the GUI |
| [test/](test/) | 31 unit tests; no robot and no ROS graph needed |

The two `*_client.py` modules and the GUI never touch the controller directly —
they go through the actions and the parameter server. That separation is the
point: one module owns the robot, everything else asks it nicely.

## Build your own controller

This package is meant to be **read and copied from**. Each row below names a
job you will hit when writing your own Cartesian node, and points at the exact
lines here that already solve it.

Paths starting `../controllers/` are in the `controllers` git submodule (the
`ros2_effort_controller` fork), so the links resolve in a clone with submodules
initialised.

### Commanding an end-effector pose

| To do this | Read |
|---|---|
| Publish one setpoint pose, correctly stamped | [`cartesian_move_server.py:160-173`](kuka_control_tutorial/cartesian_move_server.py#L160-L173) — `publish_setpoint()`, the only place in the package that writes to the topic |
| Create the publisher with a matching QoS | [`cartesian_move_server.py:89-95`](kuka_control_tutorial/cartesian_move_server.py#L89-L95) — depth 1, reliable, to match the controller's queue |
| Read the measured pose back from TF | [`cartesian_move_server.py:130-148`](kuka_control_tutorial/cartesian_move_server.py#L130-L148) — `lookup_ee_pose()`; the controller publishes no pose of its own |
| Wait for TF before commanding anything | [`cartesian_move_server.py:150-158`](kuka_control_tutorial/cartesian_move_server.py#L150-L158) — `wait_for_ee_pose()` |
| Avoid stepping the setpoint backwards between motions | [`cartesian_move_server.py:187-206`](kuka_control_tutorial/cartesian_move_server.py#L187-L206) — `start_pose()` prefers the last setpoint over the sagged measured pose |
| See where the setpoint is received and frame-checked | [`cartesian_impedance_controller.cpp:759-796`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L759-L796) — `targetFrameCallback()`, which **silently drops** poses in the wrong frame |
| See how the subscription is named | [`cartesian_impedance_controller.cpp:263-268`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L263-L268) — `<node name>/target_frame`, depth 1 |
| See a setpoint become joint torques | [`cartesian_impedance_controller.cpp:466-684`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L466-L684) — `computeTorque()`; damping is derived from `K_d` here every cycle |

Background: [doc/commanding-ee-pose.md](doc/commanding-ee-pose.md).

### Composing an action

| To do this | Read |
|---|---|
| Define goal / result / feedback | [`MoveToPose.action`](action/MoveToPose.action) (server times it), [`FollowPath.action`](action/FollowPath.action) (caller times it) |
| Register an action server | [`cartesian_move_server.py:103-121`](kuka_control_tutorial/cartesian_move_server.py#L103-L121) — two servers sharing one `ReentrantCallbackGroup` |
| Reject a bad goal before the arm moves | [`cartesian_move_server.py:379-407`](kuka_control_tutorial/cartesian_move_server.py#L379-L407) — `move_goal_callback()` |
| Validate a whole path up front | [`cartesian_move_server.py:539-599`](kuka_control_tutorial/cartesian_move_server.py#L539-L599) — `path_goal_callback()`, incl. per-segment speed |
| Check a point against a workspace envelope | [`cartesian_move_server.py:208-225`](kuka_control_tutorial/cartesian_move_server.py#L208-L225) — `check_point()` |
| Refuse concurrent motions instead of queueing | [`cartesian_move_server.py:227-244`](kuka_control_tutorial/cartesian_move_server.py#L227-L244) — `claim()` / `release()` |
| Guarantee the lock is freed on every exit | [`cartesian_move_server.py:413-417`](kuka_control_tutorial/cartesian_move_server.py#L413-L417) — `try/finally` around the body |
| Accept a cancel request | [`cartesian_move_server.py:409-411`](kuka_control_tutorial/cartesian_move_server.py#L409-L411) — plus the cooperative poll inside `stream()` |
| Stream setpoints with watchdogs and feedback | [`cartesian_move_server.py:254-336`](kuka_control_tutorial/cartesian_move_server.py#L254-L336) — `stream()`: cancel, lag abort, clock-stall abort, ~10 Hz feedback |
| Hold the target while the arm catches up | [`cartesian_move_server.py:338-375`](kuka_control_tutorial/cartesian_move_server.py#L338-L375) — `settle()` |
| Follow a complete action end to end | [`cartesian_move_server.py:430-535`](kuka_control_tutorial/cartesian_move_server.py#L430-L535) — `run_move()`, from TF lookup to `succeed()`/`abort()` |

Background: [doc/actions-and-trajectories.md](doc/actions-and-trajectories.md).

### Generating a trajectory

| To do this | Read |
|---|---|
| Build a trapezoidal velocity profile | [`trajectory.py:9-67`](kuka_control_tutorial/trajectory.py#L9-L67) — degenerates to triangular on short moves |
| Interpolate orientation along a motion | [`trajectory.py:79-100`](kuka_control_tutorial/trajectory.py#L79-L100) — `quat_slerp()`, short-way-round handled |
| Write a sampler for a straight line | [`cartesian_move_server.py:486-492`](kuka_control_tutorial/cartesian_move_server.py#L486-L492) — the `(progress, position, orientation)` contract `stream()` expects |
| Write a sampler for a timed path | [`cartesian_move_server.py:664-666`](kuka_control_tutorial/cartesian_move_server.py#L664-L666) |
| Interpolate a time-parameterised path | [`path_planning.py:182-199`](kuka_control_tutorial/path_planning.py#L182-L199) — `sample_path()` |
| Turn noisy input samples into a usable path | [`path_planning.py:140-179`](kuka_control_tutorial/path_planning.py#L140-L179) — `prepare_drawn_path()`: strip pause, thin, smooth, rescale, dwell, speed-limit |
| Slow a path to respect a speed limit | [`path_planning.py:125-137`](kuka_control_tutorial/path_planning.py#L125-L137) — uniform stretch, so the shape of the speed profile survives |
| Work out what is reachable in a plane | [`path_planning.py:202-211`](kuka_control_tutorial/path_planning.py#L202-L211) — sphere sliced at constant z gives a disc |
| Publish progress a client can use | [`cartesian_move_server.py:496-500`](kuka_control_tutorial/cartesian_move_server.py#L496-L500) — measured pose, not just a percentage |

### Making a controller parameter live-tunable

All C++, in the impedance controller fork.

| To do this | Read |
|---|---|
| Declare the parameters | [`cartesian_impedance_controller.cpp:49-54`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L49-L54) — the six `stiffness.*` doubles |
| Declare bounds and a ramp rate alongside | [`cartesian_impedance_controller.cpp:60-63`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L60-L63) |
| Register an on-set callback | [`cartesian_impedance_controller.cpp:164-168`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L164-L168) — in `on_configure`, so bounds are known |
| Validate writes so a rejection sticks | [`cartesian_impedance_controller.cpp:328-371`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L328-L371) — `onSetParameters()`; whole batch validated before any of it is latched |
| Hand values to the control loop safely | [`cartesian_impedance_controller.cpp:8-10`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L8-L10) (lock-free assertion) and [`cartesian_impedance_controller.h:162`](../controllers/cartesian_impedance_controller/include/cartesian_impedance_controller/cartesian_impedance_controller.h#L162) (atomic target array) |
| Rate-limit a change so it cannot jolt the arm | [`cartesian_impedance_controller.cpp:373-405`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L373-L405) — `updateStiffness()` |
| Apply it once per control cycle | [`cartesian_impedance_controller.cpp:408-427`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L408-L427) — `update()`, before `computeTorque()` |
| Seed the runtime value from config at startup | [`cartesian_impedance_controller.cpp:135-159`](../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp#L135-L159) |

**Never call `get_parameter()` from the update loop** — it takes a lock and is
not realtime-safe. That is the whole reason for the atomic-plus-ramp path above.

Background: [doc/dynamic-stiffness.md](doc/dynamic-stiffness.md).

### Driving a controller from Python

| To do this | Read |
|---|---|
| Wrap an action in a blocking call | [`motion_client.py:106-132`](kuka_control_tutorial/motion_client.py#L106-L132) — `_run()`: send, await accept, await result |
| Wait on a future without spinning the node | [`motion_client.py:145-151`](kuka_control_tutorial/motion_client.py#L145-L151) — `add_done_callback` + `threading.Event`, **not** `spin_until_future_complete` |
| Cancel a running goal from a client | [`motion_client.py:95-102`](kuka_control_tutorial/motion_client.py#L95-L102) |
| Set parameters on another node | [`stiffness_client.py:76-104`](kuka_control_tutorial/stiffness_client.py#L76-L104) — all six in one request so they validate as a batch |
| Read parameters from another node | [`stiffness_client.py:63-74`](kuka_control_tutorial/stiffness_client.py#L63-L74) |
| Wait for a node's parameter services | [`stiffness_client.py:48-58`](kuka_control_tutorial/stiffness_client.py#L48-L58) — both `set` and `get`; they are discovered independently |

### Wiring the build

| To do this | Read |
|---|---|
| Generate action types | [`CMakeLists.txt:14-18`](CMakeLists.txt#L14-L18) — `action_msgs` is required for actions |
| Ship Python from an `ament_cmake` package | [`CMakeLists.txt:22-31`](CMakeLists.txt#L22-L31) — installs into the rosidl-generated namespace, because a second `ament_python_install_package()` of the same name collides |
| Expose nodes to `ros2 run` | [`CMakeLists.txt:33-38`](CMakeLists.txt#L33-L38) — thin scripts in `scripts/` |
| Register ROS-free unit tests | [`CMakeLists.txt:44-47`](CMakeLists.txt#L44-L47) |
| Declare interface-package dependencies | [`package.xml:12`](package.xml#L12), [`package.xml:20`](package.xml#L20), [`package.xml:26`](package.xml#L26) — generators, runtime, `member_of_group` |

### Putting a GUI on a ROS node

| To do this | Read |
|---|---|
| Spin ROS beside a blocking UI loop | [`draw_trajectory_gui.py:669-688`](kuka_control_tutorial/draw_trajectory_gui.py#L669-L688) — executor on a thread, tk on the main thread |
| Run ROS work off the UI thread | [`draw_trajectory_gui.py:408-424`](kuka_control_tutorial/draw_trajectory_gui.py#L408-L424) — `in_worker()` |
| Marshal results back to the UI thread | [`draw_trajectory_gui.py:373-389`](kuka_control_tutorial/draw_trajectory_gui.py#L373-L389) — a queue drained by `after()`; never touch widgets from a worker |
| Map screen pixels to robot coordinates | [`draw_trajectory_gui.py:253-265`](kuka_control_tutorial/draw_trajectory_gui.py#L253-L265) |
| Draw the reachable region | [`draw_trajectory_gui.py:269-331`](kuka_control_tutorial/draw_trajectory_gui.py#L269-L331) |
| Push edits to a parameter server as the user types | [`draw_trajectory_gui.py:542-550`](kuka_control_tutorial/draw_trajectory_gui.py#L542-L550) (debounce) and [`draw_trajectory_gui.py:575-609`](kuka_control_tutorial/draw_trajectory_gui.py#L575-L609) (validate and send) |

## Build

```bash
cd ~/lbr_ws
colcon build --packages-select kuka_control_tutorial --symlink-install
source install/setup.bash
```

The GUI needs `python3-tk`, and the live stiffness needs the controller change
described in [doc/dynamic-stiffness.md](doc/dynamic-stiffness.md) (already
present in this workspace's `controllers` fork).

## Run

Start the robot with the Cartesian impedance controller:

```bash
# Gazebo
ros2 launch kuka_control gazebo.launch.py ctrl:=cartesian_impedance_controller
# or hardware
ros2 launch kuka_control hardware.launch.py model:=iiwa7 ctrl:=cartesian_impedance_controller
```

### One move

```bash
ros2 launch kuka_control_tutorial move_to_pose.launch.py use_sim_time:=true
ros2 run kuka_control_tutorial move_to_pose_client --ros-args -r __ns:=/lbr -- 0.5 0.0 0.4
```

Positions are in `lbr_link_0`, metres. Orientation is held at whatever the end
effector has when the goal starts, unless you pass `--orientation QX QY QZ QW`.
Drop `use_sim_time` on hardware.

### The drawing demo

```bash
ros2 launch kuka_control_tutorial move_to_pose.launch.py use_sim_time:=true gui:=true
```

The arm moves smoothly to `home_position` (default `-0.396, 0.472, 0.405`, where
the iiwa7 spawns in Gazebo) unless already within `home_tolerance`. A 30 × 30 cm
square opens, centred there in the XY plane at that height:

- screen right is **+X**, screen up is **+Y**, grid squares are 5 cm;
- the dark red region is **outside the arm's reach** at this height — the
  envelope check is a sphere about the base, so slicing it at constant z gives a
  circle, and with the default home pose the `-X/+Y` corner falls outside it.
  Strokes crossing it are refused before the arm moves;
- the teal ring is the measured end-effector position at 10 Hz;
- the right-hand panel is live stiffness — see
  [doc/dynamic-stiffness.md](doc/dynamic-stiffness.md).

Draw a stroke with one press-drag-release. On release the arm moves to where the
stroke began, then traces it at `speed_ratio` (default 0.2) of the speed you
drew it. **Stop** cancels; **Go home** re-centres.

## Things measured on this setup

Numbers from Gazebo with the iiwa7, at the 200 N/m translational stiffness in
`kuka_control/config/gazebo_controllers.yaml`. They are here because each one
changed a default or corrected an assumption.

**The arm settles short of its setpoint, and that is correct.** Gravity and the
spring balance at a non-zero offset. 0.08 m moves settled 0.010–0.014 m short,
which is why `goal_tolerance` defaults to 0.02 m rather than 0.01 — at 0.01 the
first run passed by 1/10 of a millimetre and later runs would have aborted
perfectly good moves.

**Traced shapes come out smaller than commanded.** Commanding a 5 cm-radius
circle:

| Circle period | Speed | Traced radius | Fidelity |
|---|---|---|---|
| 6 s | 0.052 m/s | 0.0316 m | 63 % |
| 12 s | 0.026 m/s | 0.0359 m | 72 % |
| 30 s | 0.010 m/s | 0.0395 m | 79 % |
| 60 s | 0.005 m/s | 0.0410 m | 82 % |

Slowing down helps but plateaus near 82 %. At 0.005 m/s the motion is
quasi-static, so what remains is not tracking lag — it is the static spring
offset, confirmed by sweeping stiffness on the same 5 cm move:

| `trans_*` | Settle error | Offset under a held command |
|---|---|---|
| 50 N/m | 0.0199 m | 0.0167 m |
| 200 N/m | 0.0115 m | 0.0054 m |
| 800 N/m | 0.0034 m | 0.0017 m |

**So if drawings come out small, raise the stiffness in the panel rather than
lowering the speed ratio.**

## Tests

```bash
colcon test --packages-select kuka_control_tutorial
colcon test-result --all --test-result-base build/kuka_control_tutorial
```

31 tests over the velocity profile (endpoints, monotonicity, rest-to-rest,
symmetry, triangular degeneration, argument validation), the quaternion helpers
(slerp endpoints, short-way-round, unit length) and the stroke conditioning
(thinning, smoothing, pause stripping, speed scaling and limiting,
interpolation, reach geometry). None of them need a robot.

## Verified against

Gazebo with the iiwa7, and on the real arm:

- straight-line moves in both directions, and rejection outside the envelope;
- `follow_path` tracing a 60-waypoint circle, plus rejection of a path starting
  too far away and one leaving the envelope;
- the lag watchdog, by freezing the arm mid-motion;
- the clock-stall abort, with the world paused via
  `gz service -s /world/kuka_world/control --req 'pause: true'`;
- the GUI end to end with synthetic mouse events: a 41-waypoint stroke drawn in
  1.4 s replayed over 6.8 s at 0.2x;
- live stiffness: negative, out-of-range and NaN values refused with the
  parameter left untouched; a 50 → 800 N/m edit arriving as a ~1.9 s ramp,
  matching the 400 N/m per second slew rate.

## Troubleshooting

| Symptom | Look at |
|---|---|
| Arm does not move at all | [Checklist](doc/commanding-ee-pose.md#checklist-when-the-arm-will-not-move) |
| "Got target pose in wrong reference frame" | [The frame rule](doc/commanding-ee-pose.md#the-frame-rule) |
| Goal rejected immediately | Server log — it always says which check failed |
| "settled X m away" abort | Stiffness too low for `goal_tolerance`; see [dynamic-stiffness](doc/dynamic-stiffness.md#choosing-a-value) |
| Stiffness edits do nothing | Stock controller reads stiffness only in `on_configure`; needs the fork change |
| Client hangs forever | An action path that never called `succeed`/`abort`/`canceled` |
