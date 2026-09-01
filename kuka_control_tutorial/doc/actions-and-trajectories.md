# Composing an action and a trajectory

Two actions live in this package and they share almost all of their code. This
page explains the shape they have in common, so you can read either one — or
add a third.

## Why an action and not a topic or service

| | Fits? |
|---|---|
| **Topic** | No. Fire-and-forget; you never learn whether the arm arrived. |
| **Service** | No. Blocks for the whole motion, no progress, no cancel. |
| **Action** | Yes. Seconds-long, progress while it runs, cancellable, one clear terminal state. |

A Cartesian move takes seconds, you want to watch it, and you want a stop
button. That is the exact shape an action is for.

## Anatomy of a `.action` file

Three sections separated by `---`, in a fixed order:

```
# ---- Goal: what the caller asks for ----
geometry_msgs/Point target_position
float64 speed
bool hold_orientation
geometry_msgs/Quaternion target_orientation
---
# ---- Result: one message, when it ends ----
geometry_msgs/Pose final_pose
float64 position_error
bool success
string message
---
# ---- Feedback: repeatedly, while it runs ----
geometry_msgs/Pose current_pose
float64 distance_remaining
float64 percent_complete
```

Design notes worth copying:

- **Put a `string message` in the result.** The status code tells you *that* it
  failed; the message tells you *why*, in words a user can act on. Every abort
  path in this package fills it in with the actual numbers.
- **Feedback should carry measured state, not just a percentage.** Our feedback
  includes the measured pose, so a client can plot the real arm, not the plan.
- **Make the zero value sensible.** `speed: 0.0` means "use the server default"
  rather than "do not move", so `ros2 action send_goal` with a minimal message
  does something reasonable.

The two files: [`MoveToPose.action`](../action/MoveToPose.action) (server picks
the timing) and [`FollowPath.action`](../action/FollowPath.action) (caller
supplies the timing).

## Build wiring

Interfaces need `rosidl`, which needs CMake — so this is an `ament_cmake`
package even though every node is Python.

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "action/FollowPath.action"
  DEPENDENCIES action_msgs geometry_msgs      # action_msgs is required for actions
)
```

**The gotcha.** `rosidl_generate_interfaces` already installs a Python package
named `kuka_control_tutorial`. Adding the usual
`ament_python_install_package(${PROJECT_NAME})` alongside it fails the build:

```
add_custom_target cannot create target "ament_cmake_python_symlink_kuka_control_tutorial"
because another target with the same name already exists.
```

The fix used here is to install the hand-written modules *into* the generated
package instead, so both live in one importable namespace:

```cmake
install(FILES
  ${PROJECT_NAME}/cartesian_move_server.py
  ...
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
```

Then `from kuka_control_tutorial.action import MoveToPose` and
`from kuka_control_tutorial.trajectory import ...` both work. The alternative is
a separate `..._msgs` package; this keeps it to one.

Verify after building:

```bash
ros2 interface show kuka_control_tutorial/action/MoveToPose
ros2 action list          # /lbr/move_to_pose, /lbr/follow_path
```

## The server's three callbacks

```python
ActionServer(
    self, MoveToPose, "move_to_pose",
    goal_callback=self.move_goal_callback,      # 1. accept or reject
    execute_callback=self.execute_move,         # 2. do the work
    cancel_callback=self.cancel_callback,       # 3. allow cancellation
    callback_group=ReentrantCallbackGroup(),
)
```

### 1. `goal_callback` — reject before anything moves

Everything cheap and checkable happens here, so a bad goal costs nothing:
finite numbers, inside `max_reach`, above `min_z`, under `max_speed`, and
something actually subscribed to `target_frame`. Rejecting is free; aborting
halfway is not.

This is also where the single-motion lock is taken (`claim()`). Two concurrent
motions would fight over one setpoint topic, so a second goal is **rejected,
not queued** — with a log line saying so.

### 2. `execute_callback` — and always release the lock

```python
def execute_move(self, goal_handle):
    try:
        return self.run_move(goal_handle)
    finally:
        self.release()          # even if run_move raises
```

Every path out of `run_move` must end in exactly one of `goal_handle.succeed()`,
`.abort()` or `.canceled()`, and then return a result. Miss it and the client
hangs forever waiting for a result that never comes.

### 3. `cancel_callback`

Returns `CancelResponse.ACCEPT`. The actual stopping is cooperative: the
streaming loop polls `goal_handle.is_cancel_requested` every cycle and freezes
the setpoint where it is.

## The sampler: how a trajectory plugs in

This is the key abstraction. Both actions share one streaming loop,
[`stream()`](../kuka_control_tutorial/cartesian_move_server.py) at
`cartesian_move_server.py:254`, which knows nothing about *what* path is being
followed. It asks a function:

```python
sampler(elapsed) -> (progress, position, orientation)
#   elapsed:     seconds of ROS time since the motion started
#   progress:    0.0 .. 1.0, for feedback
#   position:    (x, y, z) to command right now
#   orientation: (qx, qy, qz, qw) to command right now
```

`stream()` then owns everything that is the same for every motion:

- publishing at `control_rate` with the right frame;
- the cancel check;
- the **lag watchdog** — measured pose more than `max_lag` behind the setpoint
  means collision, joint limit, e-stop or a dead controller, so abort and drop
  the setpoint onto the measured pose so the force decays;
- the **clock-stall watchdog** — ROS time not advancing for
  `clock_stall_timeout` wall seconds means a paused sim, so abort rather than
  spin forever;
- feedback at ~10 Hz.

Add a new motion type and you get all of that for free. You write a sampler.

### `MoveToPose`: a trapezoidal profile

The trajectory is a straight line; the only question is *when* you are where
along it. `TrapezoidalProfile`
([`trajectory.py`](../kuka_control_tutorial/trajectory.py)) answers that: ramp
up at `acceleration`, cruise at `speed`, ramp down — degenerating to a
triangular profile when the move is too short to reach cruise speed.

```python
profile = TrapezoidalProfile(travel, speed, acceleration)

def sampler(elapsed):
    progress = profile.progress_at(elapsed)          # 0 .. 1, S-shaped in time
    return (progress,
            interpolate(start_xyz, target_xyz, progress),   # lerp along the line
            quat_slerp(start_quat, target_quat, progress))  # slerp, same parameter

outcome, measured, message = self.stream(
    goal_handle, sampler, profile.duration, publish_feedback)
```

Trapezoidal rather than constant velocity because the arm starts and ends at
rest. A velocity step at t=0 yanks a spring that is already under tension.

### `FollowPath`: caller-supplied timing

Here the caller hands over `waypoints` plus a `time_from_start` for each, and
the sampler is a lookup:

```python
def sampler(elapsed):
    progress = min(1.0, elapsed / duration)
    return progress, sample_path(points, times, elapsed), path_quat
```

`sample_path` ([`path_planning.py`](../kuka_control_tutorial/path_planning.py))
finds the bracketing pair of waypoints and interpolates linearly between them,
clamping outside the range.

Because the timing is the caller's, `follow_path` **refuses to start** unless
the arm is already within `path_start_tolerance` of waypoint 0 — there is no
time budget in the path for travelling to its own beginning. Send a
`move_to_pose` goal there first. The GUI does exactly that.

## After streaming: settling

Streaming ends when the interpolation ends, but the arm is still catching up.
`settle()` (`cartesian_move_server.py:338`) holds the final setpoint until the
measured pose is within `goal_tolerance`, up to `settle_timeout`.

Under a soft stiffness the arm stops *short* of the setpoint — gravity and the
spring balance at a non-zero offset. That is correct impedance behaviour, not a
failure, which is why `goal_tolerance` defaults to 0.02 m and not something
tighter. See [dynamic-stiffness.md](dynamic-stiffness.md) for the measurements.

`move_to_pose` aborts if it never settles; `follow_path` succeeds and reports
the offset, because a traced shape is about the path, not about nailing the
final point.

## Calling an action from your own code

The raw API is asynchronous, which is awkward when you want to run motions in
order. [`MotionClient`](../kuka_control_tutorial/motion_client.py) wraps both
actions in blocking calls:

```python
from kuka_control_tutorial.motion_client import MotionClient

client = MotionClient(node)
client.wait_for_servers()

if client.move_to(path[0], speed=0.05):      # Outcome is falsy on failure
    client.follow_path(path, times)
```

**Call these from a thread that is not spinning the node.** The blocking wait is:

```python
done = threading.Event()
future.add_done_callback(lambda _: done.set())
done.wait(timeout)
```

not `rclpy.spin_until_future_complete()`. Spinning the node from inside a
callback that the executor is already running deadlocks. The pattern here is:
`MultiThreadedExecutor` spins on one thread, your sequence runs on another and
waits on events.

## Recipe: adding a third action

1. Write `action/YourAction.action` — goal, result, feedback.
2. Add it to `rosidl_generate_interfaces` in `CMakeLists.txt`.
3. In `CartesianMoveServer.__init__`, add an `ActionServer` with its three
   callbacks and the **same** `ReentrantCallbackGroup`.
4. `goal_callback`: validate with `check_point()` for each point, then `claim()`.
5. `execute_callback`: `try: ... finally: self.release()`.
6. In the body: `wait_for_ee_pose()` → `start_pose()` → build a `sampler` →
   `self.stream(...)` → `self.settle(...)` → succeed/abort/cancel.
7. Add a blocking wrapper in `MotionClient` if callers want one.
8. Put the maths in a ROS-free module and unit-test it, as
   `trajectory.py` and `path_planning.py` do — 31 of the tests in this package
   need no robot and no ROS graph.

## Trying the actions by hand

```bash
# with feedback
ros2 action send_goal -f /lbr/move_to_pose kuka_control_tutorial/action/MoveToPose \
  "{target_position: {x: -0.396, y: 0.472, z: 0.455}, speed: 0.03, hold_orientation: true}"

ros2 action list -t
ros2 action info /lbr/move_to_pose
```
