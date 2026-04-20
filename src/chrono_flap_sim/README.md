# chrono_flap_sim

ROS 2 C++ package that runs a **Project Chrono** multibody simulation of an inverted-pendulum
flap on a revolute joint. The node supports two operating modes — **SIL** (Software-in-the-Loop,
no hardware required) and **parallel/shadow** (runs alongside real ODrive hardware for model
validation) — and exposes all physical parameters for online reconfiguration via `rqt_reconfigure`.

---

## Overview

### Physical setup

- 30 cm × 30 cm × 0.25 cm acrylic flap (~0.5 kg identified mass)
- Pivot at the **bottom edge** (inverted pendulum — flap stands upright)
- Gravity acts in −Z; revolute joint rotates about the Y axis
- **Powered ODrive** on one end of the pivot axis drives the flap
- **Unpowered ODrive** on the other end acts as a passive bearing with friction

### Operating modes

| Mode | `sil_mode` | Publishes `/joint_states` | Role |
|---|---|---|---|
| **SIL** | `true` | ✓ at `rate_hz` | Acts as the plant; PID closes loop through simulation |
| **Parallel** | `false` (default) | ✗ | Shadows the real hardware; publishes `~/sim_*` for comparison |

---

## Physics model

The flap is modelled as a thin rectangular plate pivoting at its bottom edge (inverted pendulum).

### Inertia

For a uniform rectangular plate of mass *m* and height *L*, with the pivot at the bottom edge:

```
I_pivot = (1/3) · m · L²
```

### Torque law

Each solver sub-step applies:

```
τ_total = τ_external − (B_joint + B_bearing) · ω − C_coulomb · sign(ω) − K · θ
```

| Symbol | Parameter | Description |
|---|---|---|
| `τ_external` | — | Torque from `velocity_pid_node` via the effort topic (N·m) |
| `B_joint` | `joint_damping` | Viscous damping at the powered ODrive joint (N·m·s/rad) |
| `B_bearing` | `bearing_friction` | Bearing friction at the unpowered ODrive (N·m·s/rad) |
| `C_coulomb` | `coulomb_friction` | Coulomb (dry) friction magnitude (N·m); constant braking force opposing motion at all speeds |
| `K` | `joint_stiffness` | Restoring spring stiffness (N·m/rad); models cable/structural stiffness |
| `ω` | — | Instantaneous joint angular velocity (rad/s) |
| `θ` | — | Joint angle measured from upright equilibrium (rad) |

Gravity is handled by Project Chrono through the rigid-body simulation; it does not appear
explicitly in the torque law above.

---

## Operating modes

### SIL mode (`sil_mode:=true`)

`chrono_flap_node` publishes `sensor_msgs/JointState` on `/joint_states`, replacing
`joint_state_broadcaster`. The `velocity_pid_node` closes the control loop through the
simulation. No ODrive, CAN bus, or motor is needed.

```
chrono_flap_node (sil_mode=true)
      │  /joint_states (100 Hz)
      ▼
velocity_pid_node ──/motor_effort_controller/commands──▶ chrono_flap_node
```

**Typical launch:**

The easiest way to start SIL mode is the dedicated launch file, which starts
`robot_state_publisher`, `chrono_flap_node` (with `sil_mode:=true`), and `velocity_pid_node`
together:

```bash
ros2 launch chrono_flap_sim sil_mode.launch.py
```

Optional arguments:

```bash
ros2 launch chrono_flap_sim sil_mode.launch.py \
  bearing_friction:=0.3 \
  control_mode:=cascade \
  position_setpoint:=0.8 \
  enable_visualization:=false
```

### RViz visualization (SIL mode)

With the launch file running, open RViz in a second terminal to visualize the robot model:

```bash
# In a second terminal
rviz2
```

In RViz:

1. Set **Fixed Frame** to `base_link`
2. **Add** → **RobotModel** → set **Description Source** to "Topic" and **Description Topic** to `/robot_description`
3. **Add** → **TF** (optional, to see frame axes)

The URDF includes visual geometry for all three links: grey `base_link` cube, blue translucent
`motor_link` flap (0.0025 m thick × 0.30 m wide × 0.30 m tall, swings in the XZ plane about
the Y axis with the pivot at the bottom edge), and orange `pto_link` cylinder.

### Alternative: manual launch (two terminals)

```bash
# Terminal 1 — simulation plant
ros2 run chrono_flap_sim chrono_flap_node --ros-args \
  -p sil_mode:=true \
  -p bearing_friction:=0.4 \
  -p joint_stiffness:=0.712441

# Terminal 2 — PID controller
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p joint_name:=motor_joint \
  -p control_mode:=cascade \
  -p position_setpoint:=0.5
```

### Parallel mode (`sil_mode:=false`, default)

The real ODrive owns `/joint_states`. `chrono_flap_node` subscribes to the same torque command
the hardware receives and integrates the equations of motion, publishing its predictions on
`~/sim_*` topics. Compare these against real hardware measurements in PlotJuggler to validate
the identified parameters.

In parallel mode, `chrono_flap_node` also subscribes to `/joint_states` (the real hardware
measurements) and applies a **Luenberger observer correction** to prevent the simulated position
from drifting away from reality over time. Each tick, after Chrono integrates, the predicted
position and velocity are nudged toward the measured values:

```
θ_corrected = θ_predicted + α · (θ_measured − θ_predicted)
ω_corrected = ω_predicted + β · (ω_measured − ω_predicted)
```

where `α` is the `observer_gain` parameter (default `0.05`) and `β` is `velocity_observer_gain`
(default `0.0`). The corrected state is fed back into the Chrono joint body so that the next
integration step (including the stiffness term `K·θ`) uses the corrected values. This preserves
fast transient dynamics (which Chrono models well) while eliminating slow drift due to open-loop
integration error. Set `observer_gain:=0.0` to disable the correction and run fully open-loop.

For a detailed theoretical explanation, see
[`doc/parallel_mode_observer.md`](doc/parallel_mode_observer.md).

```
ODrive HW ──/joint_states──▶ velocity_pid_node ──/motor_effort_controller/commands──▶ ODrive HW
               │                                               │
               │ (measured position for observer)              ▼
               └──────────────────────────────▶ chrono_flap_node (sil_mode=false)
                                                ~/sim_position, ~/sim_velocity, ~/sim_acceleration
```

**Typical launch (via launch file):**

```bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

The launch file starts `chrono_flap_node` automatically in parallel mode. To enable the 3D
visualization window:

```bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py enable_visualization:=true
```

---

## Parameters

Parameters marked **Immutable** require a node restart; **Reconfigurable** parameters can be
changed at runtime via `ros2 param set` or `rqt_reconfigure`.

| Parameter | Type | Default | Reconfigurable | Description |
|---|---|---|---|---|
| `sil_mode` | bool | `false` | ✗ | Publish `sensor_msgs/JointState` on `/joint_states` for SIL closed-loop operation |
| `joint_name` | string | `motor_joint` | ✗ | Joint name written into the `JointState` message |
| `joint_state_topic` | string | `/joint_states` | ✗ | Topic on which `/joint_states` is published (SIL only) |
| `rate_hz` | double | `100.0` | ✗ | Publish and control-loop rate (Hz) |
| `solver_rate_hz` | double | `1000.0` | ✗ | Internal Chrono solver rate (Hz); solver runs `solver_rate_hz / rate_hz` sub-steps per publish tick |
| `effort_topic` | string | `/motor_effort_controller/commands` | ✗ | Torque input topic (`std_msgs/Float64MultiArray`, first element used) |
| `enable_visualization` | bool | `false` | ✗ | Enable Chrono 3D visualization window (requires a Chrono build with VSG or Irrlicht support) |
| `flap_length_m` | double | `0.30` | ✓ | Flap height/length (m); updates pivot-to-CoM distance and inertia |
| `flap_width_m` | double | `0.30` | ✓ | Flap width (m); updates inertia tensor |
| `flap_mass_kg` | double | `0.5` | ✓ | Flap mass (kg); identified value from parameter ID |
| `joint_damping` | double | `0.0` | ✓ | Viscous damping at the powered ODrive joint (N·m·s/rad) |
| `joint_stiffness` | double | `0.712441` | ✓ | Restoring spring stiffness (N·m/rad); identified value from parameter ID |
| `bearing_friction` | double | `0.4` | ✓ | Bearing friction at the unpowered ODrive (N·m·s/rad); identified test-bench value |
| `coulomb_friction` | double | `0.0` | ✓ | Coulomb (dry) friction magnitude (N·m); constant braking force applied as `coulomb_friction * sign(ω)`. Models static/kinetic dry friction in bearings. |
| `observer_gain` | double | `0.05` | ✓ | Luenberger observer gain α ∈ [0.0, 1.0] (parallel mode only). Each tick: `θ_corrected = θ_predicted + α·(θ_measured − θ_predicted)`. `0.0` = fully open-loop (will drift); `1.0` = snap to measurement each tick. See [`doc/parallel_mode_observer.md`](doc/parallel_mode_observer.md). |
| `velocity_observer_gain` | double | `0.0` | ✓ | Luenberger velocity observer gain β ∈ [0.0, 1.0] (parallel mode only). Each tick: `ω_corrected = ω_predicted + β·(ω_measured − ω_predicted)`. Corrected velocity is fed back into the Chrono body so the stiffness term uses the right speed on the next tick. `0.0` = no velocity correction (default). See [`doc/parallel_mode_observer.md`](doc/parallel_mode_observer.md). |

> **Identified values:** For the 30 cm × 30 cm acrylic flap on this test bench, the parameter
> identification results are:
>
> | Parameter | Default (code) | Identified hardware value |
> |---|---|---|
> | `flap_mass_kg` | `0.5` | 0.5 kg |
> | `joint_stiffness` | `0.712441` | 0.712441 N·m/rad |
> | `bearing_friction` | `0.4` | **0.2 N·m·s/rad** |
> | `joint_damping` | `0.0` | 0.0 N·m·s/rad |
>
> Always pass `bearing_friction:=0.4` (the current default) when running on this hardware.
> See the root [`README.md`](../../README.md#identified-plant-parameters) for the full parameter identification table.

---

## Published topics

| Topic | Type | Condition | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | SIL mode only (`sil_mode=true`) | Simulated joint position and velocity for both `motor_joint` and `pto_joint` (pto_joint is published with zero position/velocity/effort); replaces `joint_state_broadcaster`, enabling `robot_state_publisher` to compute TFs for all links |
| `~/sim_position` | `std_msgs/Float64` | Always | Simulated joint angle (rad) |
| `~/sim_velocity` | `std_msgs/Float64` | Always | Simulated joint angular velocity (rad/s) |
| `~/sim_acceleration` | `std_msgs/Float64` | Always | Simulated joint angular acceleration (rad/s²) |

All `~/` topics are scoped under the node name (e.g. `/chrono_flap_node/sim_position`).

---

## Subscribed topics

| Topic | Type | Condition | Description |
|---|---|---|---|
| `/motor_effort_controller/commands` | `std_msgs/Float64MultiArray` | Always | Torque input (N·m); first element used |
| `/joint_states` | `sensor_msgs/JointState` | Parallel mode only (`sil_mode=false`) | Real hardware joint states; position and velocity of `joint_name` are used for the Luenberger observer correction |

---

## PlotJuggler comparison (parallel mode)

After launching the full hardware stack, subscribe to the following topics in PlotJuggler to
compare the Chrono shadow prediction against real measurements:

| Simulated (Chrono) | Real (hardware) | Description |
|---|---|---|
| `/chrono_flap_node/sim_position` | `/velocity_pid_node/measured_position` | Joint angle (rad) |
| `/chrono_flap_node/sim_velocity` | `/velocity_pid_node/measured_velocity` | Joint velocity (rad/s) |

If the traces track closely, the identified parameters are a good model of the plant. Divergence
indicates where the model needs refinement (e.g. unmodelled friction, structural flexibility).

---

## Prerequisites

- [Project Chrono](https://projectchrono.org) — core library (`ChronoEngine`).
  Optional: VSG (`Chrono_vsg`) or Irrlicht (`Chrono_irrlicht`) module for 3D visualization.
- ROS 2 Jazzy (or compatible).

---

## Building

```bash
# Headless (core Chrono only — always works):
colcon build --packages-select chrono_flap_sim

# With VSG visualization (if Chrono was built with VSG support):
colcon build --packages-select chrono_flap_sim \
  --cmake-args -DCMAKE_PREFIX_PATH=/path/to/chrono/install
```

`CMakeLists.txt` automatically detects `Chrono_vsg` / `Chrono_irrlicht` targets and adds the
appropriate compile definition (`CHRONO_VSG` or `CHRONO_IRRLICHT`). If neither is found, the node
builds without visualization support; attempting `enable_visualization:=true` at runtime will log
a warning and continue headlessly.

