# Commanding an end-effector pose

Everything this package does to the robot happens through **one topic**. If you
read nothing else, read this page.

## The topic

| | |
|---|---|
| **Topic** | `/lbr/cartesian_impedance_controller/target_frame` |
| **Type** | `geometry_msgs/msg/PoseStamped` |
| **`header.frame_id`** | **must** be `lbr_link_0` |
| **QoS** | reliable, depth 1 |
| **Direction** | we publish, the controller subscribes |

The controller creates it in
[`cartesian_impedance_controller.cpp:263`](../../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp),
as `<node name>/target_frame` with a queue depth of 1. The node runs in the
`lbr` namespace, hence the full path above.

Check it is really there before debugging anything else:

```bash
ros2 topic info /lbr/cartesian_impedance_controller/target_frame
# Type: geometry_msgs/msg/PoseStamped
# Publisher count: 1        <- the action server
# Subscription count: 1     <- the controller
```

A subscription count of 0 means the controller is not loaded or not active. The
action server checks exactly this before accepting a goal, so you get a clear
rejection instead of commanding into the void.

## What the controller does with it

The pose is not a waypoint to travel to. It is the **anchor of a spring**. The
controller computes an error between the current end-effector pose and this
target, multiplies by the stiffness matrix, and turns that into joint torques
(`computeTorque()`).

That single fact drives the whole design:

> A setpoint 30 cm away does not mean "go there over the next few seconds". It
> means "you are 30 cm from equilibrium **right now**" — and at 200 N/m that is
> 60 N of pull, applied immediately.

So we never publish a distant target. We publish a *stream* of nearby ones that
walk to the goal, keeping the spring short and the force bounded. That is the
entire justification for the action servers in this package.

## The frame rule

`targetFrameCallback`
([`cartesian_impedance_controller.cpp:759`](../../controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp))
begins with:

```cpp
if (target->header.frame_id != Base::m_robot_base_link) {
  RCLCPP_WARN_THROTTLE(..., "Got target pose in wrong reference frame. ...");
  return;                      // the setpoint is dropped
}
```

Get the frame wrong and your poses are **silently discarded** — bar a throttled
warning that is easy to miss in a busy log. The arm simply does not move. This
is the single most common way to waste an afternoon here.

`m_robot_base_link` comes from the controller's `robot_base_link` parameter,
`lbr_link_0` in `kuka_control/config/*.yaml`. Our publisher stamps that frame
from the `base_link` parameter, so **the two must agree**; if you change one,
change the other.

## Namespacing gotcha

`kuka_control/launch/hardware.launch.py` remaps `target_frame` to a short name
for *some* controllers:

```python
remappings=[
    ("joint_impedance_controller/target_frame", "target_frame"),
    ("kuka_clik_controller/target_frame",       "target_frame"),
],
```

`cartesian_impedance_controller` is **not** in that list, so its topic keeps the
fully qualified name. Do not assume `/lbr/target_frame` works — it does not, for
this controller. If you add a remap later, update the `target_frame_topic`
parameter to match.

## Where this happens in the code

There is exactly **one** function in this package that writes to the topic:

**[`cartesian_move_server.py:160`](../kuka_control_tutorial/cartesian_move_server.py) — `publish_setpoint()`**

```python
def publish_setpoint(self, position, orientation):
    msg = PoseStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.base_link      # the frame rule, in one line
    ...
    self.target_pub.publish(msg)
    self._last_setpoint = (position, orientation)
```

Keeping it to one chokepoint is deliberate. It means the frame is stamped
correctly in one place, and it means `_last_setpoint` always reflects what the
robot was actually told — which the next goal relies on (see below).

Everything that moves the arm calls it:

| Caller | File | Why |
|---|---|---|
| `stream()` | `cartesian_move_server.py:254` | the per-cycle setpoint during any motion |
| `stream()` on cancel | `cartesian_move_server.py` | re-publishes the last setpoint to freeze in place |
| `stream()` on lag abort | `cartesian_move_server.py` | publishes the **measured** pose so the spring force decays |
| `settle()` | `cartesian_move_server.py:338` | holds the final target while the arm catches up |
| `run_move()` zero-distance case | `cartesian_move_server.py:430` | already at the target |

Nothing else in the package — not the GUI, not `MotionClient`, not the CLI
client — touches the topic. They all go through the actions. If you are adding
a feature and find yourself about to publish a `PoseStamped`, you almost
certainly want to call an action instead.

## Reading the pose back

The controller publishes **no** end-effector pose. To know where the arm
actually is, we read TF:

```
lbr_link_0  ->  lbr_link_ee
```

via `lookup_ee_pose()`
([`cartesian_move_server.py:130`](../kuka_control_tutorial/cartesian_move_server.py)),
which is published by `robot_state_publisher` from `/joint_states`. From a
terminal:

```bash
ros2 run tf2_ros tf2_echo lbr_link_0 lbr_link_ee
```

The gap between commanded setpoint and measured pose is meaningful, not noise —
it *is* the spring extension, and therefore the force. Two safety features read
it directly:

- **the lag watchdog**, which aborts if the measurement falls more than
  `max_lag` behind the setpoint (collision, joint limit, e-stop, controller
  deactivated);
- **`start_pose()`** ([`cartesian_move_server.py:187`](../kuka_control_tutorial/cartesian_move_server.py)),
  which starts a new motion from the *previous setpoint* rather than the
  measured pose when the two still agree. Starting from the measured pose would
  step the setpoint backwards by the standing spring extension at the beginning
  of every goal.

## Commanding it by hand

Useful for a first sanity check. **Publish close to where the arm already is** —
remember the spring:

```bash
# Look first
ros2 run tf2_ros tf2_echo lbr_link_0 lbr_link_ee

# Then nudge, a centimetre or two from the current pose
ros2 topic pub --once /lbr/cartesian_impedance_controller/target_frame \
  geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "lbr_link_0"},
    pose: {position: {x: -0.396, y: 0.472, z: 0.415},
           orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}'
```

Watch what is being commanded during a motion:

```bash
ros2 topic echo /lbr/cartesian_impedance_controller/target_frame
ros2 topic hz   /lbr/cartesian_impedance_controller/target_frame   # ~100 Hz
```

## Rate

We stream at `control_rate`, 100 Hz by default. The controller runs its update
loop at `update_rate` (1000 Hz in `kuka_control/config/*.yaml`) and holds the
last setpoint between our messages, so we do not need to match it — the spring
interpolates for us. Publishing much faster buys nothing; publishing much slower
makes the setpoint visibly staircase.

## Checklist when the arm will not move

1. `ros2 topic info` — is the subscription count 1?
2. Is `header.frame_id` exactly `lbr_link_0`?
3. `ros2 control list_controllers -c /lbr/controller_manager` — is the
   controller `active`?
4. Is the stiffness non-zero? At 0.05 N/m the arm is effectively free.
   See [dynamic-stiffness.md](dynamic-stiffness.md).
5. `ros2 run tf2_ros tf2_echo lbr_link_0 lbr_link_ee` — is TF alive? Without it
   the action server aborts rather than moving blind.
