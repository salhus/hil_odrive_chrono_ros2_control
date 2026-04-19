# hil_odrive_ros2_control (ROS 2 Jazzy)

This repository is a self-contained ROS 2 Jazzy workspace implementing a **Wave Energy Converter (WEC) Hardware-in-the-Loop (HIL) dynamometer** test bench. Two ODrive motors on a shared shaft are controlled over **SocketCAN (CAN bus)** via **ros2_control**:

- **Motor 1 (Hydro Emulator, axis0, `node_id=0`)** — driven by `velocity_pid_node` to replay wave-driven shaft motion (sine-wave velocity trajectory)
- **Motor 2 (PTO / Power Take-Off, axis1, `node_id=1`)** — passively resists shaft motion with `τ = -B · ω`. In Phase 1 the ODrive's onboard velocity controller handles the damping; no extra ROS control node is needed.

It is designed so you can:

- clone this repo into a fresh ROS 2 Jazzy workspace (`~/ws/src`)
- `colcon build`
- launch `ros2_control_node` + controllers
- run the velocity PID node and command the hydro emulator motor

---

## WEC HIL dyno concept

```
                    ┌─────────────────────┐
  can0 ◄───────────┤   ODrive (1 board)   │
                    │                     │
                    │  axis0 (node_id=0)  │──── Motor 1: Hydro Emulator (motor_joint)
                    │                     │         ║
                    │                     │     shared shaft
                    │                     │         ║
                    │  axis1 (node_id=1)  │──── Motor 2: PTO passive damper (pto_joint)
                    └─────────────────────┘
```

Both motors share the same ODrive board and CAN bus (`can0`). The second joint (`pto_joint`) is registered inside the same `<ros2_control name="ODriveSystem">` hardware block in the URDF.

---

## What's in this repo

### Packages

- **`hil_odrive_ros2_control`** (root package)
  - Purpose: installs the launch file, controller YAML, and URDF/Xacro for `motor_joint` (axis0) and `pto_joint` (axis1)
  - Key paths:
    - `launch/motor_control.launch.py`
    - `config/controllers.yaml`
    - `description/urdf/motor.urdf.xacro`

- **`odrive_velocity_pid`** (package)
  - Purpose: reads measured velocity from `/joint_states`, tracks a sine-wave reference velocity for `motor_joint`, runs a PID loop, and publishes torque (effort) commands

### Upstream ODrive packages

The following packages are sourced from the upstream [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository and live under `src/` as sibling colcon packages:

- `src/odrive_ros2_control` (ros2_control hardware interface plugin)
- `src/odrive_base` (base code used by the hardware plugin)

See [`VENDORED.md`](VENDORED.md) for provenance and licensing details.

---

## High-level architecture / data flow

### Runtime components

1. **`ros2_control_node`** (from `controller_manager`)
   - loads the ODrive hardware plugin declared in the URDF
   - exposes ros2_control state and command interfaces for `motor_joint` (axis0) and `pto_joint` (axis1)

2. **`joint_state_broadcaster`**
   - publishes `sensor_msgs/msg/JointState` on `/joint_states`
   - provides measured velocity feedback for both joints

3. **`motor_effort_controller`**
   - accepts `std_msgs/msg/Float64MultiArray` commands on `/motor_effort_controller/commands`
   - forwards them to the ros2_control effort command interface for `motor_joint`

4. **`pto_effort_controller`**
   - accepts `std_msgs/msg/Float64MultiArray` commands on `/pto_effort_controller/commands`
   - forwards them to the ros2_control effort command interface for `pto_joint`
   - In Phase 1 this controller is spawned but not actively commanded from ROS — the ODrive's onboard velocity controller handles passive damping on axis1

5. **`odrive_velocity_pid/velocity_pid_node`**
   - a **standalone ROS 2 node** (not a ros2_control controller plugin)
   - subscribes to `/joint_states`
   - extracts measured position and velocity for `motor_joint`
   - generates a sine trajectory reference:  
     **pos_ref(t) = position_setpoint + (A/ω)·(1 − cos(ω·t))**  
     **vel_ref(t) = A·sin(ω·t)**  
     **accel_ref(t) = A·ω·cos(ω·t)**
   - runs a cascaded PID (configurable mode: `position_only`, `cascade`, or `velocity_only`)
   - publishes torque command as `std_msgs/msg/Float64MultiArray` to `/motor_effort_controller/commands`

6. **`chrono_flap_sim/chrono_flap_node`** (included in `motor_control.launch.py`)
   - runs a Project Chrono inverted-pendulum flap simulation in **parallel/shadow** mode by default
   - subscribes to `/motor_effort_controller/commands` and integrates the equations of motion
   - publishes `~/sim_position`, `~/sim_velocity`, `~/sim_acceleration` for model validation
   - does **not** publish `/joint_states` in parallel mode (the real ODrive owns that topic)
   - see [`src/chrono_flap_sim/README.md`](../chrono_flap_sim/README.md) for the full physics model and parameter reference

### Data flow summary

```
/joint_states (position + velocity) → velocity_pid_node → /motor_effort_controller/commands (effort) → effort controller → ODrive ros2_control hardware plugin → CAN → ODrive axis0 (motor_joint)

ODrive axis1 (pto_joint) ← passive damping τ = -B·ω configured via odrivetool

/motor_effort_controller/commands → chrono_flap_node (parallel mode) → ~/sim_position, ~/sim_velocity
```

Note: `velocity_pid_node` is a standalone node — it is **not** a ros2_control controller plugin.
It reads from the joint state broadcaster's output topic and writes directly to the effort
controller's command topic.

> **SIL alternative:** For development without hardware, use the dedicated launch file:
> ```bash
> ros2 launch chrono_flap_sim sil_mode.launch.py
> ```
> This starts `robot_state_publisher`, `chrono_flap_node` (with `sil_mode:=true`), and
> `velocity_pid_node` together — no ODrive, CAN bus, or motor required. See the root
> [`README.md`](../../README.md#quick-start-sil-mode-no-hardware) for full details and RViz
> visualization instructions.

---

## Prerequisites

### ROS 2 Jazzy
You must have ROS 2 Jazzy installed. Make sure you source it in every terminal before building/running:

```bash
source /opt/ros/jazzy/setup.bash
```

### ros2_control + controllers
You need the controller manager and controllers installed (commonly via `ros-jazzy-ros2-control` and `ros-jazzy-ros2-controllers`).

In particular, this repo expects an effort controller type:

- `effort_controllers/JointGroupEffortController`

If you see controller-type load failures, install the appropriate Jazzy packages and/or check available controller types (see troubleshooting).

### SocketCAN + can-utils
You need a Linux SocketCAN interface (e.g. `can0`). Helpful tools:

```bash
sudo apt-get install can-utils
```

---

## SocketCAN setup (example)

ODrive CAN bitrate depends on your configuration. Example with `250000` (250k):

```bash
# Bring interface down if it exists
sudo ip link set can0 down 2>/dev/null || true

# Configure bitrate and bring it up
sudo ip link set can0 up type can bitrate 250000

# Inspect interface state/details
ip -details link show can0
```

Verify traffic:

```bash
candump can0
```

If `candump` shows nothing, common causes include: wrong bitrate, wiring/termination issues, ODrive not transmitting, or wrong interface name.

---

## Configuration you MUST check (CAN + node_ids)

The ODrive hardware plugin configuration is in:

- `description/urdf/motor.urdf.xacro`

Key parameters:

### CAN interface name
```xml
<param name="can">can0</param>
```

### ODrive CAN node IDs
```xml
<!-- Motor 1: Hydro Emulator (ODrive axis0) -->
<joint name="motor_joint">
  <param name="node_id">0</param>
</joint>

<!-- Motor 2: PTO passive damper (ODrive axis1) -->
<joint name="pto_joint">
  <param name="node_id">1</param>
</joint>
```

Make sure:
- your SocketCAN interface is actually `can0` (or change it)
- ODrive `node_id=0` is axis0 (hydro emulator) and `node_id=1` is axis1 (PTO)

---

## PTO motor configuration (Phase 1 — passive linear damper)

Configure ODrive axis1 directly via `odrivetool` so it resists shaft motion proportionally to velocity:

```python
# In odrivetool
odrv0.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_setpoint = 0
odrv0.axis1.controller.config.vel_gain = B   # damping coefficient B (Nm·s/rad)
odrv0.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
```

The velocity controller's P-gain acts as damping coefficient `B`. When Motor 1 spins the shaft at ω, axis1 applies `τ = -B · ω`.

### Power telemetry via ros2_control state interfaces

The hardware plugin exposes `electrical_power` and `mechanical_power` state interfaces for both
joints, populated from ODrive `Get_Powers` CAN broadcast messages. However, the ODrive does
**not** broadcast these by default. Enable the broadcast rate via `odrivetool`:

```python
# In odrivetool — enable power telemetry broadcast on both axes (10 Hz)
odrv0.axis0.config.can.get_powers_msg_rate_ms = 100
odrv0.axis1.config.can.get_powers_msg_rate_ms = 100
odrv0.save_configuration()
```

Once configured, `electrical_power` and `mechanical_power` will be non-NaN in `/dynamic_joint_states`.
This replaces the need to probe `V_bus × I_bus` on the DC bus with an oscilloscope.

---

## Build (fresh workspace)

```bash
# Terminal
source /opt/ros/jazzy/setup.bash

mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/salhus/hil_odrive_ros2_control.git
cd ..

# Install dependencies
rosdep install --from-paths src -y --ignore-src

# Build (headless — no Chrono visualization)
colcon build --symlink-install

# Source overlay
source install/setup.bash
```

If Project Chrono was built with VSG (or Irrlicht) visualization support, pass its install
prefix so CMake can find it:

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_PREFIX_PATH=/path/to/chrono/install
```

### Confirm packages are discoverable by colcon
```bash
colcon list
```

You should see packages like:
- `hil_odrive_ros2_control`
- `chrono_flap_sim`
- `odrive_velocity_pid`
- `odrive_ros2_control`
- `odrive_base`

---

## Run: bring up ros2_control + controllers

Launch the WEC HIL dyno setup:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash

ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

This launch file starts:
- `ros2_control_node`
- `robot_state_publisher`
- spawns/activates:
  - `joint_state_broadcaster`
  - `motor_effort_controller` (Motor 1, hydro emulator)
  - `pto_effort_controller` (Motor 2, PTO — spawned but passively controlled by ODrive in Phase 1)
- `velocity_pid_node` (cascade mode, `amplitude_rad_s=0.25`, `omega_rad_s=0.25`)
- `chrono_flap_node` (parallel/shadow mode by default)

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `controllers_file` | `config/controllers.yaml` | Path to controller YAML |
| `enable_visualization` | `false` | Enable Chrono 3D visualization window (requires Vulkan/GPU and a Chrono build with VSG or Irrlicht) |

To enable the Chrono 3D visualization window:

```bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py enable_visualization:=true
```

---

## Verify: controllers, interfaces, feedback

### Check controllers
```bash
ros2 control list_controllers
```

You want all three of these to show as `active`:
- `joint_state_broadcaster`
- `motor_effort_controller`
- `pto_effort_controller`

### Check hardware interfaces
```bash
ros2 control list_hardware_interfaces
```

Look for both `motor_joint` and `pto_joint` interfaces, including state interfaces for
`position`, `velocity`, `effort`, `electrical_power`, and `mechanical_power`.

### Check feedback stream
```bash
ros2 topic echo /joint_states --once
```

Confirm:
- Both `motor_joint` and `pto_joint` appear in `name: [...]`
- `velocity: [...]` has sensible values (not NaN) for both joints

### Check power telemetry
```bash
ros2 topic echo /dynamic_joint_states
```

`electrical_power` and `mechanical_power` appear here (not on `/joint_states`) once the ODrive
is configured to broadcast `Get_Powers` messages. See the
[power telemetry configuration](#power-telemetry-via-ros2_control-state-interfaces) section.

---

## Run: velocity PID node (hydro emulator — sine trajectory → torque)

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash

ros2 run odrive_velocity_pid velocity_pid_node
```

The node starts in `position_only` mode with trajectory amplitude and frequency set to `0.0`
(stationary). Override parameters with `--ros-args -p key:=value`:

```bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p control_mode:=cascade \
  -p amplitude_rad_s:=0.25 \
  -p omega_rad_s:=0.25 \
  -p torque_limit_nm:=0.4
```

> **Architecture note:** `VelocityPidNode` is a **standalone ROS 2 node** — it is *not* a
> ros2_control controller plugin. It subscribes to `/joint_states` for feedback and publishes
> directly to the effort controller topic. This means it can be started, stopped, and tuned
> independently of the controller manager.

### Control modes

| Mode | Description |
|---|---|
| `position_only` *(default)* | Outer position PID → torque directly. Good for commissioning. |
| `cascade` | Outer position PID → velocity command → inner velocity PID → torque. Best for trajectory tracking. |
| `velocity_only` | Single velocity PID loop. Backward-compatible flat-PID behaviour. |

Switch mode at runtime:

```bash
ros2 param set /velocity_pid_node control_mode cascade
```

### PID node parameters (`odrive_velocity_pid`)

#### Immutable (require node restart)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_state_topic` | string | `/joint_states` | JointState feedback topic |
| `command_topic` | string | `/motor_effort_controller/commands` | Effort command output topic |
| `joint_name` | string | `motor_joint` | Joint name inside `/joint_states.name[]` |
| `rate_hz` | double | `100.0` | Control-loop frequency (Hz) |

#### Runtime-reconfigurable

| Parameter | Type | Default | Description |
|---|---|---|---|
| `control_mode` | string | `position_only` | Active control mode: `position_only`, `cascade`, or `velocity_only` |
| `amplitude_rad_s` | double | `0.0` | Sine trajectory amplitude (rad/s in velocity_only; rad in cascade/position_only). `0.0` = stationary. |
| `omega_rad_s` | double | `0.0` | Sine angular frequency (rad/s). For 1 Hz use `2π ≈ 6.283`. `0.0` = stationary. |
| `position_setpoint` | double | `0.0` | Static position setpoint (rad). Sine oscillates around this value. |
| `kp` | double | `0.35` | Inner-loop proportional gain |
| `ki` | double | `0.01` | Inner-loop integral gain |
| `kd` | double | `0.0` | Inner-loop derivative gain |
| `kff` | double | `0.40` | Velocity feedforward gain (suppressed when `kp=0`) |
| `kaff` | double | `0.20` | Acceleration feedforward gain (suppressed when `kp=0`) |
| `torque_limit_nm` | double | `0.40` | Output torque saturation limit (N·m) |
| `integral_limit` | double | `0.0` | Inner-loop integral clamp. Must be positive to take effect. |
| `deadband_rad_s` | double | `0.0` | Velocity error deadband |
| `kp_pos` | double | `2.0` | Outer-loop proportional gain |
| `ki_pos` | double | `0.01` | Outer-loop integral gain |
| `kd_pos` | double | `0.025` | Outer-loop derivative gain |
| `pos_integral_limit` | double | `1.0` | Outer-loop integral clamp (must be positive) |
| `pos_output_limit` | double | `2.0` | Max velocity command from the outer loop (rad/s) |
| `outer_loop_divider` | double | `1.0` | Run outer loop every N inner-loop ticks |
| `filter_alpha` | double | `0.90` | Velocity EMA smoothing coefficient (`[0.0, 1.0)`) |
| `invert_output` | bool | `false` | Negate torque and flip measured signs |

---

## Troubleshooting

### 1) CAN interface down / no traffic
Symptoms:
- No `/joint_states` updates (or they remain zero/NaN)
- `candump can0` shows nothing

Checks/fixes:
- `ip -details link show can0`
- confirm bitrate matches the bus
- check wiring and termination
- confirm the ODrive is powered and connected

### 2) Wrong ODrive node ID
Symptoms:
- CAN is up, but hardware plugin never gets valid feedback / joint stays NaN

Fix:
- Update `node_id` values in `description/urdf/motor.urdf.xacro` to match the ODrive CAN node IDs

### 3) Controllers not active
Symptoms:
- PID publishes torque commands but motor doesn't respond
- `/motor_effort_controller/commands` exists but effort interface not claimed

Checks:
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Fix:
- make sure `motor_effort_controller` and `pto_effort_controller` are `active` (spawner should do this)
- verify controller manager is running and reachable at `/controller_manager`

### 4) Effort controller type not available
Symptoms:
- controller fails to load with a type-not-found error

Checks:
```bash
ros2 control list_controller_types
```

Fix:
- install `ros-jazzy-ros2-controllers` (or equivalent)
- update `config/controllers.yaml` to use an effort controller type that exists on your system

### 5) `pto_joint` missing from `/joint_states`
Symptoms:
- `/joint_states` publishes but `pto_joint` isn't present

Fix:
- confirm axis1 on the ODrive is calibrated and set to `CLOSED_LOOP_CONTROL` via `odrivetool`
- check `ros2_control_node` logs for hardware/plugin load errors for `node_id=1`

### 6) ODrive not in a state that accepts torque commands
Symptoms:
- feedback exists but motor does not move/respond

Fix:
- ensure the ODrive is calibrated/configured for closed-loop control
- confirm it accepts CAN setpoints as expected for your ODrive firmware/config

---

## Phase 2 (planned)

Phase 2 will add a **pluggable PTO control framework** for comparing WEC control strategies on the same hardware bench:

| Strategy | Law |
|---|---|
| Passive damping (baseline) | `τ = -B·ω` |
| Optimal passive | `τ = -B_opt·ω` (B_opt matches radiation damping) |
| Reactive (complex conjugate) | `τ = -B·ω - K·x` |
| Latching | Lock shaft at extremes, release at optimal phase |
| Declutching | Free shaft periodically, engage at optimal phase |
| MPC | Model-predictive with wave prediction horizon |

---

## Licensing / vendored code

The `src/odrive_base/` and `src/odrive_ros2_control/` packages are sourced from the upstream [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository and retain its MIT license.

Provenance and update notes are documented in:

- [`VENDORED.md`](VENDORED.md)
