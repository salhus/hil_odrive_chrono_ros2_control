# odrive_velocity_pid

A ROS 2 Jazzy C++ package that implements a **cascaded PID controller** for a single joint
(`motor_joint` by default). It subscribes to `/joint_states`, generates a sinusoidal trajectory
reference, and publishes torque (effort) commands.

> **Architecture note:** `VelocityPidNode` is a **standalone ROS 2 node**. It does *not* run
> as a ros2_control controller plugin inside the controller manager — it subscribes to
> `/joint_states` for feedback and publishes directly to an effort controller topic. This is an
> intentional design choice that keeps the PID logic simple and independently restartable.

The node works identically whether `/joint_states` is published by real hardware
(`joint_state_broadcaster` + ODrive) or by `chrono_flap_node` with `sil_mode:=true`. This makes
it straightforward to develop and tune the controller entirely in simulation before connecting
hardware.

---

## SIL mode (no hardware)

When `chrono_flap_node` is launched with `sil_mode:=true`, it publishes `sensor_msgs/JointState`
on `/joint_states` at `rate_hz` Hz. The `velocity_pid_node` consumes these messages exactly as it
would from a real motor — the interface is identical.

The easiest way to start SIL mode is the dedicated launch file:

```bash
ros2 launch chrono_flap_sim sil_mode.launch.py
```

This starts `robot_state_publisher`, `chrono_flap_node` (with `sil_mode:=true`), and
`velocity_pid_node` together. Optional arguments:

```bash
ros2 launch chrono_flap_sim sil_mode.launch.py \
  bearing_friction:=0.3 \
  control_mode:=position_only \
  position_setpoint:=0.8 \
  enable_visualization:=false
```

### Alternative: manual launch (two terminals)

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

Verify data is flowing:

```bash
ros2 topic hz /joint_states
ros2 topic echo /velocity_pid_node/measured_position --once
```

Use `rqt_reconfigure` to adjust PID gains and physical parameters without restarting either node:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

---

## Prerequisites

- ROS 2 Jazzy desktop installed and sourced
- A running `ros2_control` stack with a `joint_state_broadcaster` and an effort controller
  (e.g. `effort_controllers/JointGroupEffortController`) for the hardware side, **or**
  `chrono_flap_node` with `sil_mode:=true` for simulation (no hardware required)

---

## Build

```bash
# From your ROS 2 workspace root (e.g. ~/ws)
colcon build --packages-select odrive_velocity_pid
source install/setup.bash
```

---

## Control modes

Three modes are selected at startup via the `control_mode` parameter. The mode can also be
changed **at runtime** via `ros2 param set` or `rqt_reconfigure`.

### `position_only` (default)

The outer position PID drives torque directly. Useful for tuning position gains and commissioning
before enabling the full cascade.

```
pos_ref → [position PID] → torque (clamped)
```

### `cascade` (recommended for trajectory tracking)

Two-loop cascade: outer position PID feeds a velocity command to the inner velocity PID.
Analytical velocity and acceleration derivatives of the sine trajectory are used as feedforwards
at each level. Proper bandwidth separation between the loops gives good stability and disturbance
rejection.

```
pos_ref → [position PID] → v_cmd → [velocity PID + kff·v_ref + kaff·a_ref] → torque
```

### `velocity_only`

Single velocity-tracking loop. The sine trajectory produces a velocity setpoint directly.
Equivalent to the original flat-PID behaviour.

```
vel_ref → [velocity PID + kff·v_ref + kaff·a_ref] → torque
```

---

## Run

Start the node with default parameters:

```bash
ros2 run odrive_velocity_pid velocity_pid_node
```

Override specific parameters at launch with `--ros-args -p key:=value`:

```bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p control_mode:=cascade \
  -p amplitude_rad_s:=0.25 \
  -p omega_rad_s:=0.25 \
  -p torque_limit_nm:=0.4
```

Change parameters **at runtime** (most parameters are reconfigurable without restarting):

```bash
ros2 param set /velocity_pid_node control_mode cascade
ros2 param set /velocity_pid_node kp 0.4
ros2 param set /velocity_pid_node amplitude_rad_s 0.5
```

Or use `rqt_reconfigure` for a GUI:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

---

## ROS Parameters

### Immutable parameters (require node restart to change)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_state_topic` | `string` | `/joint_states` | Topic for `sensor_msgs/JointState` feedback |
| `command_topic` | `string` | `/motor_effort_controller/commands` | Topic for `std_msgs/Float64MultiArray` torque output |
| `joint_name` | `string` | `motor_joint` | Name of the joint inside `JointState.name[]` |
| `rate_hz` | `double` | `100.0` | Control loop rate (Hz) |

### Runtime-reconfigurable parameters

#### Trajectory

| Parameter | Type | Default | Description |
|---|---|---|---|
| `amplitude_rad_s` | `double` | `0.0` | Sine trajectory amplitude. In `velocity_only` mode: rad/s. In `cascade`/`position_only` modes: rad (peak position excursion = `amplitude / omega`). `0.0` = stationary (hold position setpoint). |
| `omega_rad_s` | `double` | `0.0` | Sine angular frequency (rad/s). `0.0` = stationary. For 1 Hz use `2π ≈ 6.283`. |
| `position_setpoint` | `double` | `0.0` | Static position setpoint (rad). The sine trajectory oscillates around this value in cascade/position_only modes. |

#### Control mode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `control_mode` | `string` | `position_only` | Active control mode: `position_only`, `cascade`, or `velocity_only`. |

#### Inner loop (velocity PID)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp` | `double` | `0.35` | Proportional gain |
| `ki` | `double` | `0.01` | Integral gain |
| `kd` | `double` | `0.0` | Derivative gain |
| `kff` | `double` | `0.40` | Velocity feedforward gain — scales the velocity reference to produce anticipatory torque (compensates viscous friction / back-EMF). Suppressed when `kp = 0`. |
| `kaff` | `double` | `0.20` | Acceleration feedforward gain — scales the acceleration reference to produce anticipatory torque (compensates rotor inertia, `kaff ≈ J`). Suppressed when `kp = 0`. |
| `torque_limit_nm` | `double` | `0.40` | Output torque saturation limit (N·m). Must be positive. |
| `integral_limit` | `double` | `0.0` | Inner-loop integral accumulator clamp. `0.0` disables the integrator (falls back to default on startup if set to a non-positive value). Must be positive to take effect. |
| `deadband_rad_s` | `double` | `0.0` | Velocity error deadband — errors smaller than this are treated as zero. |

#### Outer loop (position PID — used in `cascade` and `position_only` modes)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp_pos` | `double` | `2.0` | Proportional gain |
| `ki_pos` | `double` | `0.01` | Integral gain |
| `kd_pos` | `double` | `0.025` | Derivative gain |
| `pos_integral_limit` | `double` | `1.0` | Outer-loop integral accumulator clamp. Must be positive. |
| `pos_output_limit` | `double` | `2.0` | Maximum velocity command from the outer loop (rad/s). Must be positive. |
| `outer_loop_divider` | `double` | `1.0` | Run the outer loop every N inner-loop ticks (rate division). `1.0` = same rate as inner loop. Must be positive. |

#### Miscellaneous

| Parameter | Type | Default | Description |
|---|---|---|---|
| `filter_alpha` | `double` | `0.90` | Exponential moving average coefficient applied to velocity feedback (`0.0` = no filter, approaching `1.0` = heavier smoothing). Must be in `[0.0, 1.0)`. |
| `invert_output` | `bool` | `false` | Negate the torque command and flip the sign of measured position/velocity — set `true` if positive torque produces negative velocity on your hardware. |

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `command_topic` (default `/motor_effort_controller/commands`) | `std_msgs/Float64MultiArray` | Torque command sent to the effort controller |
| `~/desired_velocity` | `std_msgs/Float64` | Sine-wave reference velocity (rad/s) |
| `~/measured_velocity` | `std_msgs/Float64` | Filtered encoder feedback velocity (rad/s) |
| `~/velocity_error` | `std_msgs/Float64` | Velocity tracking error (rad/s) |
| `~/measured_position` | `std_msgs/Float64` | Measured joint position from `/joint_states` (rad) |
| `~/position_error` | `std_msgs/Float64` | Position tracking error: reference − measured (rad) |
| `~/position_command` | `std_msgs/Float64` | Position reference sent to the outer loop (rad) |
| `~/velocity_command` | `std_msgs/Float64` | Velocity command output from the outer loop (rad/s) |

The `~/` prefix scopes topics under the node name (e.g. `/velocity_pid_node/desired_velocity`).
These topics are ideal for plotting in PlotJuggler or `rqt_plot`.

---

## Console Debug Logging

The control loop emits a throttled `INFO` log at ~10 Hz (100 ms interval):

```
[cascade] pos_err=0.003 v_cmd=1.234 vel_err=0.056 torque=0.120 sat=0
```

| Field | Description |
|---|---|
| `[mode]` | Active control mode (`cascade`, `position_only`, or `velocity_only`) |
| `pos_err` | Position tracking error (rad) |
| `v_cmd` | Current velocity command from the outer loop (rad/s) |
| `vel_err` | Velocity tracking error (rad/s) |
| `torque` | Torque command after saturation (N·m) |
| `sat` | `1` if the inner-loop output is currently saturated, `0` otherwise |

---

## Safety Features

- **Torque saturation** — output is clamped to `[-torque_limit_nm, torque_limit_nm]` every tick.
- **Integral clamp** — integral accumulator is clamped to `[-integral_limit, integral_limit]`.
- **Anti-windup** — directional (conditional) anti-windup: when saturated, integration is only
  allowed in the direction that *unwinds* the integrator, preventing permanent lockup.
- **Feedforward suppression** — `kff` and `kaff` are suppressed (with a logged warning) when
  `kp = 0`, preventing open-loop torque runaway.
- **Zero torque on shutdown** — the destructor publishes a zero-torque command before exiting.
- **Joint not found** — if `joint_name` is missing from `/joint_states`, zero torque is published
  every tick until the joint appears.

---

## `config/controllers.yaml`

Declares the `ros2_control` controllers used with this package:

| Controller | Type | Notes |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes joint feedback |
| `motor_effort_controller` | `effort_controllers/JointGroupEffortController` | Accepts effort commands for `motor_joint` |
| `controller_manager` | — | `update_rate: 200` Hz |
