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
τ_total = τ_external − (B_joint + B_bearing) · ω − K · θ
```

| Symbol | Parameter | Description |
|---|---|---|
| `τ_external` | — | Torque from `velocity_pid_node` via the effort topic (N·m) |
| `B_joint` | `joint_damping` | Viscous damping at the powered ODrive joint (N·m·s/rad) |
| `B_bearing` | `bearing_friction` | Bearing friction at the unpowered ODrive (N·m·s/rad) |
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

```bash
# Terminal 1 — simulation plant
ros2 run chrono_flap_sim chrono_flap_node --ros-args \
  -p sil_mode:=true \
  -p bearing_friction:=0.2 \
  -p joint_stiffness:=0.712441

# Terminal 2 — PID controller
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p joint_name:=motor_joint \
  -p control_mode:=position_only \
  -p position_setpoint:=0.5
```

### Parallel mode (`sil_mode:=false`, default)

The real ODrive owns `/joint_states`. `chrono_flap_node` subscribes to the same torque command
the hardware receives and integrates the equations of motion, publishing its predictions on
`~/sim_*` topics. Compare these against real hardware measurements in PlotJuggler to validate
the identified parameters.

```
ODrive HW ──/joint_states──▶ velocity_pid_node ──/motor_effort_controller/commands──▶ ODrive HW
                                                            │
                                                            ▼
                                                chrono_flap_node (sil_mode=false)
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
| `bearing_friction` | double | `0.005` | ✓ | Bearing friction at the unpowered ODrive (N·m·s/rad); set to `0.2` for identified hardware |

> **Identified values:** For the 30 cm × 30 cm acrylic flap on this test bench, use
> `flap_mass_kg:=0.5`, `joint_stiffness:=0.712441`, `bearing_friction:=0.2`.
> See the root [`README.md`](../../README.md#identified-plant-parameters) for the full parameter identification table.

---

## Published topics

| Topic | Type | Condition | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | SIL mode only (`sil_mode=true`) | Simulated joint position and velocity; replaces `joint_state_broadcaster` |
| `~/sim_position` | `std_msgs/Float64` | Always | Simulated joint angle (rad) |
| `~/sim_velocity` | `std_msgs/Float64` | Always | Simulated joint angular velocity (rad/s) |
| `~/sim_acceleration` | `std_msgs/Float64` | Always | Simulated joint angular acceleration (rad/s²) |

All `~/` topics are scoped under the node name (e.g. `/chrono_flap_node/sim_position`).

---

## Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/motor_effort_controller/commands` | `std_msgs/Float64MultiArray` | Torque input (N·m); first element used |

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
  --cmake-args -DChronoConfig_DIR=/path/to/chrono/lib/cmake/Chrono
```

`CMakeLists.txt` automatically detects `Chrono_vsg` / `Chrono_irrlicht` targets and adds the
appropriate compile definition (`CHRONO_VSG` or `CHRONO_IRRLICHT`). If neither is found, the node
builds without visualization support; attempting `enable_visualization:=true` at runtime will log
a warning and continue headlessly.

