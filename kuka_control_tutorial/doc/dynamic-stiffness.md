# Dynamically configurable stiffness

The end-effector stiffness can be changed **while the controller is running**.
This page lists the entry points, then explains the path a value takes from a
text box to the control law.

Stock `cartesian_impedance_controller` reads its stiffness once in
`on_configure`. Live editing needs the controller change described below, which
lives in this workspace's fork (`controllers` submodule, branch
`kuka-lbr-control`).

## The six parameters

On the controller node, `/lbr/cartesian_impedance_controller`:

| Parameter | Unit | Axis |
|---|---|---|
| `stiffness.trans_x` | N/m | translation along base X |
| `stiffness.trans_y` | N/m | translation along base Y |
| `stiffness.trans_z` | N/m | translation along base Z |
| `stiffness.rot_x` | Nm/rad | rotation about base X |
| `stiffness.rot_y` | Nm/rad | rotation about base Y |
| `stiffness.rot_z` | Nm/rad | rotation about base Z |

The axes are expressed in `stiffness_ref_link` (`lbr_link_0` in this
workspace's config), so they line up with the base frame rather than the tool.

Damping is **not** a parameter. The control law derives it from the stiffness
every cycle via `compute_correct_damping(Lambda, K_d, ...)`, so it follows any
change automatically. Do not add a damping parameter expecting it to be used;
`m_cartesian_damping` is kept consistent but the control law does not read it.

## Entry point 1 — the GUI panel

```bash
ros2 launch kuka_control_tutorial move_to_pose.launch.py gui:=true
```

Six fields down the right-hand side. They are loaded from the running controller
at startup and pushed back as you type — 400 ms after the last keystroke
(`stiffness_push_delay_ms`), or immediately on <kbd>Enter</kbd> or when the field
loses focus. The delay is why typing `200` does not travel through `2` and `20`
on the way.

Values are checked in the panel *and* in the controller. A rejected value never
reaches the control law and the parameter keeps its previous value, so the panel
and the robot cannot drift apart. **Reload** pulls the controller's current
values back into the fields, e.g. after editing from a terminal.

GUI-side code: `_build_stiffness_panel()` and `_push_stiffness()` in
[`draw_trajectory_gui.py`](../kuka_control_tutorial/draw_trajectory_gui.py).

## Entry point 2 — the command line

```bash
ros2 param get /lbr/cartesian_impedance_controller stiffness.trans_x
ros2 param set /lbr/cartesian_impedance_controller stiffness.trans_x 350.0

# all six at once
ros2 param set /lbr/cartesian_impedance_controller stiffness.trans_x 350.0 && \
ros2 param set /lbr/cartesian_impedance_controller stiffness.trans_y 350.0 && \
ros2 param set /lbr/cartesian_impedance_controller stiffness.trans_z 350.0

ros2 param list /lbr/cartesian_impedance_controller | grep stiffness
```

A refused value reports the reason:

```
$ ros2 param set /lbr/cartesian_impedance_controller stiffness.trans_x -5.0
Setting parameter failed: stiffness.trans_x must be within [0, 2000.000000]
```

## Entry point 3 — from Python

[`StiffnessClient`](../kuka_control_tutorial/stiffness_client.py), same blocking
style as `MotionClient`:

```python
from kuka_control_tutorial.stiffness_client import StiffnessClient

stiffness = StiffnessClient(node)            # defaults to /lbr/cartesian_impedance_controller
stiffness.wait_for_service()

current = stiffness.get()                    # [tx, ty, tz, rx, ry, rz] or None
ok, reason = stiffness.set([350.0, 350.0, 350.0, 10.0, 10.0, 10.0])
if not ok:
    node.get_logger().error(reason)
```

`set()` sends all six in **one** `SetParameters` request, so the controller
validates them as a batch and a bad value cannot leave the others half-applied.

Note it waits for *both* the `set_parameters` and `get_parameters` services.
They live on the same node but are discovered independently, so waiting for one
and immediately using the other loses the race.

## What happens inside the controller

Four steps, in
[`cartesian_impedance_controller.cpp`](../../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp):

### 1. Declare (`on_init`, line 49)

The six `stiffness.*` doubles, plus the bounds and ramp rates:

```cpp
auto_declare<double>("max_stiffness_lin", 2000.0);       // N/m
auto_declare<double>("max_stiffness_rot", 200.0);        // Nm/rad
auto_declare<double>("stiffness_slew_rate_lin", 400.0);  // N/m per second
auto_declare<double>("stiffness_slew_rate_rot", 50.0);   // Nm/rad per second
```

### 2. Validate (`onSetParameters`, line 328)

Registered in `on_configure` (line 164) via `add_on_set_parameters_callback`.
Runs on the node's parameter thread, **before** rclcpp stores the value:

- must be `PARAMETER_DOUBLE`, finite, and within `[0, max_stiffness_lin/rot]`;
- the whole batch is validated before any of it is latched, so a bad axis in a
  six-axis write does not leave a partial set behind;
- returning `successful = false` means rclcpp **never stores it**. That is what
  keeps `ros2 param get` and the control law in agreement — there is no way to
  have a parameter the controller is ignoring.

### 3. Hand over safely (atomics)

The callback thread writes; the control loop reads. Accepted values go into
`std::array<std::atomic<double>, 6> m_stiffness_target` rather than straight
into the control law, with a compile-time check that this costs no lock:

```cpp
static_assert(std::atomic<double>::is_always_lock_free, ...);
```

### 4. Ramp (`updateStiffness`, line 373)

Called once per control cycle from `update()` (line 415), before
`computeTorque()`. It moves the working stiffness toward the target by at most
`stiffness_slew_rate_* × dt`:

```cpp
const double step = (rate > 0.0) ? std::min(std::abs(delta), rate * dt)
                                 : std::abs(delta);
m_stiffness_current[i] += std::copysign(step, delta);
```

Ramping is the safety-critical part. Raising the stiffness while the end
effector sits a centimetre off its setpoint would otherwise **step** the
commanded torque: 1 cm at 800 N/m is 8 N appearing in one cycle. It also makes a
torn read across the six atomics harmless — a transiently mixed target is just a
momentarily different ramp destination, corrected on the next cycle.

Set a slew rate to 0 or below to apply changes instantly, if you know what you
are doing.

Measured: a 50 → 800 N/m edit completed in ~1.9 s, matching 750 N/m at
400 N/m per second.

## Choosing a value

Measured in Gazebo, repeating a 5 cm move at each stiffness:

| `trans_*` | Settle error | Offset under a held command | Feel |
|---|---|---|---|
| 0.05 N/m | — | never converges | free / gravity-comp-like |
| 50 N/m | 0.0199 m | 0.0167 m | very compliant |
| 200 N/m | 0.0115 m | 0.0054 m | the workspace default |
| 800 N/m | 0.0034 m | 0.0017 m | stiff, accurate |

The offset is the spring extension needed to hold the arm against gravity, so it
scales roughly as 1/stiffness. Two consequences:

- **Accuracy is a stiffness choice, not a speed choice.** Traced shapes come out
  ~18 % small at 200 N/m and slowing down barely helps — at 0.005 m/s the motion
  is quasi-static, so what remains is static offset. Raise the stiffness.
- **`goal_tolerance` must be looser than the offset**, or moves abort having
  done nothing wrong. 0.02 m suits 200 N/m; tighten it if you raise stiffness.

Two other parameters interact with stiffness:

- `max_lag` (0.15 m) — the collision watchdog threshold. Too tight and normal
  offset trips it; too loose and a real collision goes unnoticed. Scale with
  stiffness.
- `delta_tau_max` (`effort_controller_base`) — caps the torque change per cycle.
  A second safety net under the slew rate.

> **On hardware**, `kuka_control/config/controllers.yaml` is what is loaded, not
> `gazebo_controllers.yaml`. Both currently start at 200 N/m translational, but
> they are separate files and drift apart easily — check the one you are
> actually running before expecting a goal to converge. The 0.05 N/m row above
> is what `controllers.yaml` used to hold, and is a good illustration of why a
> goal can time out having done nothing wrong.

## Recipe: making another parameter live-tunable

1. `auto_declare` it in `on_init`, plus any bound it needs.
2. Add an `std::atomic<T>` member for the target, and a plain member for the
   value the control loop uses.
3. Extend `onSetParameters` to recognise the name, validate it, and store to the
   atomic. Validate the whole batch before latching any of it.
4. If a step change would move the robot, ramp it in `update()` the way
   `updateStiffness` does. If it cannot (a frame name, a boolean mode), consider
   whether it should be live-tunable at all — some things belong in
   `on_configure`.
5. Read it from the control law only through the ramped member, never by calling
   `get_parameter()` in the update loop: that takes a lock and is not
   realtime-safe.
