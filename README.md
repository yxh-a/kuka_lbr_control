# KUKA LBR Control  

A repository for controlling the KUKA lbr iiwa or med with cartesian impedance control using ROS2.

<table>
    <tr>
        <th>ROS2 Distro</td>
        <th>Controller</td>
        <th>FRI library</th>
        <th>LBR Stack</th>
    </tr>
    <tr>
        <td>Humble</td>
        <td><a href='humble-controllers'><img src='https://github.com/lucabeber/effort_controller/actions/workflows/humble.yml/badge.svg'></a><br/> </td>
        <td><a href='humble-fri-library'><img src='https://github.com/lbr-stack/fri/actions/workflows/build.yaml/badge.svg?branch=fri-1.15'></a><br/> </td>
        <td><a href='humble-lbr-stack'><img src='https://github.com/idra-lab/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.15.yml/badge.svg'></a><br/> </td>
    </tr>
    <td>Jazzy</td>
        <td><a href='jazzy-controllers'><img src='https://github.com/lucabeber/effort_controller/actions/workflows/jazzy.yml/badge.svg'></a><br/> </td>
        <td><a href='jazzy-fri-library'><img src='https://github.com/lbr-stack/fri/actions/workflows/build.yaml/badge.svg?branch=fri-1.15'></a><br/> </td>
        <td><a href='jazzy-lbr-stack'><img src='https://github.com/idra-lab/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-24.04-fri-1.15.yml/badge.svg'></a><br/></td>
    </tr>
</table>

## Install

Developed and run on **Ubuntu 24.04 / ROS 2 Jazzy** with **Gazebo Harmonic**.

> **Note on distro:** the `lbr_fri_ros2_stack` and `fri` submodules currently
> track upstream *Humble* branches, and `.github/workflows/main.yml` still
> targets Humble. The sources build and run against Jazzy in practice, but this
> mismatch is unresolved -- see [Known issues](#known-issues).

### 1. Clone into a workspace

This repository *is* the workspace `src` directory, so clone it **as** `src`:

```bash
mkdir -p ~/lbr_ws
git clone --recursive https://github.com/yxh-a/kuka_lbr_control.git ~/lbr_ws/src
```

Already cloned without `--recursive`?

```bash
cd ~/lbr_ws/src && git submodule update --init --recursive
```

### 2. Install dependencies

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers

cd ~/lbr_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
cd ~/lbr_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

Gazebo simulation (loads `kuka_control/config/gazebo_controllers.yaml` and
`kuka_control/config/world.sdf`):

```bash
ros2 launch kuka_control gazebo.launch.py model:=iiwa7 ctrl:=cartesian_impedance_controller
```

Real hardware:

```bash
ros2 launch kuka_control hardware.launch.py model:=iiwa7 ctrl:=cartesian_impedance_controller
```

Useful arguments for both: `model` (`iiwa7`, `iiwa14`, `med7`, `med14`),
`robot_name`, `ctrl`, and `sys_cfg`. `gazebo.launch.py` additionally accepts
`world`. `sys_cfg` defaults to the torque system config for the impedance and
gravity-compensation controllers, and to the position config otherwise.

## Build your own controller

`kuka_control_tutorial` is written to be read and copied from. Its
**[Build your own controller](kuka_control_tutorial/README.md#build-your-own-controller)**
section indexes the jobs you hit when writing a Cartesian node — commanding an
end-effector pose, composing an action, generating a trajectory, making a
controller parameter live-tunable, driving a controller from Python, wiring the
build — against the exact file and line range in this workspace that already
solves each one.

Three background guides go with it:

- [Commanding an end-effector pose](kuka_control_tutorial/doc/commanding-ee-pose.md)
- [Composing an action and a trajectory](kuka_control_tutorial/doc/actions-and-trajectories.md)
- [Dynamically configurable stiffness](kuka_control_tutorial/doc/dynamic-stiffness.md)

## Relationship to upstream

This is a fork. It carries fixes that are not yet upstream, across three repos:

| Component | Fork | Upstream |
|---|---|---|
| Workspace + `kuka_control` | `yxh-a/kuka_lbr_control` | [idra-lab/kuka_lbr_control](https://github.com/idra-lab/kuka_lbr_control) |
| Effort / impedance controllers | `yxh-a/ros2_effort_controller` @ `kuka-lbr-control` | [idra-lab/ros2_effort_controller](https://github.com/idra-lab/ros2_effort_controller) |
| LBR FRI ROS 2 stack | `yxh-a/lbr_fri_ros2_stack` @ `humble-kuka-lbr-control` | [lbr-stack/lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) |

`fri` and `lib_fri_idl` are unmodified and still point at `lbr-stack`.

Each fork keeps the original as an `upstream` remote, so changes can be pulled in:

```bash
cd controllers        # or lbr-stack/lbr_fri_ros2_stack
git fetch upstream
git rebase upstream/main   # upstream/humble for the LBR stack
```

### What the fork fixes

- **Gazebo could not use workspace controllers.** `lbr_gazebo.xacro` hardcoded
  `lbr_description`'s `lbr_controllers.yaml`. A `gazebo_controllers_path` xacro
  argument is now threaded from the robot xacro down to the gz_ros2_control
  plugin.
- **Torque-mode simulation had no effort interface.** In gazebo mode the
  description always emitted a position command interface only; it now emits the
  interface matching `client_command_mode`.
- **`compliance_ref_link` was dead code.** The parameter and its chain
  validation were commented out upstream, so impedance was always referenced to
  the chain tip. Both are restored and the FK/Jacobian are evaluated at that
  segment, with a new `stiffness_ref_link` to choose the frame the stiffness
  axes are expressed in.
- **The controller killed the process on transient faults.** A non-finite torque
  or a large effort step called `std::terminate()`. It now holds the previous
  command and warns (throttled).
- **Null-space stiffness was a single scalar** for all joints; it is now a
  per-joint list, validated against the joint count.
- **Cartesian stiffness could only be set at configure time.** The six
  `stiffness.*` parameters were read once in `on_configure`, so changing the
  end-effector stiffness meant reloading the controller. They are now accepted
  while the controller runs, through a validating `on_set_parameters` callback
  and a slew-rate-limited ramp in the update loop, so a live edit cannot step
  the commanded torque. See `kuka_control_tutorial/README.md` for the details
  and the new `max_stiffness_*` / `stiffness_slew_rate_*` parameters.

## Known issues

- `kuka_control/config/initial_joint_positions.yaml` is consumed as **degrees**
  (`lbr_system_interface.xacro` multiplies by `PI/180`) but currently holds
  radian values, so the arm spawns at roughly 1/57th of the intended angles.
- Submodules pin upstream *Humble* branches while the working environment is
  Jazzy; CI in `.github/workflows/main.yml` also still targets Humble.
- `.gitmodules` pins `fri` to branch `fri-1.17`, but the checked-out commit is
  from `fri-1.15`.

## Run the controllers on real hardware
### Kinematics controls
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/kin.gif' width="640"/>
</div>

---

### Gravity Compensation
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/grav.gif' width="640"/>
</div>

---

### Cartesian Impedance Control with Null Space Task
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/imp.gif' width="640"/>
</div>

---  

## Gazebo Simulation
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/gazebo.gif' width="640"/>
</div>

---
---  
## Citation
If you use these controllers, please consider citing our work and leaving us a star to support the project. :mechanical_arm: 🫶
```
@article{nardi2026anatomy,
  author={Nardi, Davide and Lamon, Edoardo and Fontanelli, Daniele and Saveriano, Matteo and Palopoli, Luigi},
  journal={IEEE Robotics and Automation Letters}, 
  title={An Anatomy-Aware Shared Control Approach for Assisted Teleoperation of Lung Ultrasound Examinations}, 
  year={2026},
  volume={11},
  number={3},
  pages={2570-2577},
  keywords={Robots;Ribs;Probes;Ultrasonic imaging;Skin;Solid modeling;Computational modeling;Three-dimensional displays;Biological system modeling;Cameras;Medical robots and systems;physical human-robot interaction;telerobotics and teleoperation},
  doi={10.1109/LRA.2026.3653292}}
```


## References
- [Cartesian controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git)
- [lbr ROS2 stack (KUKA Hardware Interface)](https://github.com/lbr-stack/lbr_fri_ros2_stack)
