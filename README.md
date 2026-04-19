# hil_odrive_ros2_control

A self-contained **ROS 2 Jazzy** workspace implementing a **Wave Energy Converter (WEC) Hardware-in-the-Loop (HIL) dynamometer** test bench using two ODrive motors on a shared shaft, controlled over **SocketCAN (CAN bus)** via **ros2_control**.

**Motor 1 (Hydro Emulator, axis0)** — driven by the existing `velocity_pid_node` to replay wave-driven shaft motion (sine-wave velocity trajectory).  
**Motor 2 (PTO / Power Take-Off, axis1)** — passively resists shaft motion with `τ = -B · ω` (linear damper). In Phase 1 the damping is configured directly on the ODrive; no extra ROS control node is needed.

A Project Chrono multibody simulation (`chrono_flap_node`) runs alongside the control stack in either **SIL** (Software-in-the-Loop, no hardware required) or **parallel/shadow** mode.

---

## Dual-mode operation

Two operating modes are supported, selectable via the `sil_mode` parameter on `chrono_flap_node`:

### SIL mode (`sil_mode:=true`) — no hardware required

`chrono_flap_node` acts as the plant: it publishes `sensor_msgs/JointState` on `/joint_states`, and `velocity_pid_node` closes the control loop entirely through the simulation. No ODrive, CAN bus, or motor is needed.

```
chrono_flap_node (sil_mode=true)
        │  publishes /joint_states
        ▼
velocity_pid_node ──/motor_effort_controller/commands──▶ chrono_flap_node
```

### Parallel mode (`sil_mode:=false`, default) — hardware shadow

The real ODrive owns `/joint_states` via `joint_state_broadcaster`. `chrono_flap_node` reads the same torque commands the real hardware receives but does **not** publish `/joint_states`. Simulated and measured kinematics can be compared side-by-side in PlotJuggler to validate the identified plant model.

```
ODrive HW ──/joint_states──▶ velocity_pid_node ──/motor_effort_controller/commands──▶ ODrive HW
                                                              │
                                                              ▼
                                                  chrono_flap_node (sil_mode=false)
                                                  publishes ~/sim_position, ~/sim_velocity,
                                                            ~/sim_acceleration
```

---

## Quick start: SIL mode (no hardware)

```bash
# Terminal 1 — Chrono simulation (acts as the plant)
ros2 run chrono_flap_sim chrono_flap_node --ros-args -p sil_mode:=true -p bearing_friction:=0.2

# Terminal 2 — PID controller
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p joint_name:=motor_joint \
  -p control_mode:=position_only \
  -p position_setpoint:=0.5
```

Verify data is flowing:

```bash
ros2 topic hz /joint_states
ros2 topic echo /chrono_flap_node/sim_position --once
```

---

## Quick start: parallel mode (with hardware)

```bash
# Terminal 1 — CAN bus
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 250000

# Terminal 2 — Hardware stack (ros2_control + ODrive + PID + Chrono shadow)
ros2 launch hil_odrive_ros2_control motor_control.launch.py

# Terminal 3 — Chrono shadow (optional, if not started by launch file)
ros2 run chrono_flap_sim chrono_flap_node --ros-args -p bearing_friction:=0.2
```

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

  velocity_pid_node ──τ_wave──▶ motor_effort_controller ──▶ ODrive axis0
  (sine wave, hydro emulator)

  ODrive axis1 configured as passive damper via odrivetool
  (velocity mode, setpoint = 0, P-gain = damping coefficient B)

  Power measurement: electrical_power + mechanical_power state interfaces (via Get_Powers CAN broadcast)
```

Both motors are on the **same ODrive board** (`can0`). axis0 = `node_id=0` (hydro emulator), axis1 = `node_id=1` (PTO).

---

## Repository structure

```
hil_odrive_ros2_control/
└── src/
    ├── chrono_flap_sim/             # Project Chrono multibody simulation (SIL + parallel shadow)
    ├── hil_odrive_ros2_control/     # Launch files, controller YAML, URDF/Xacro
    ├── odrive_base/                 # ODrive base library (vendored from odriverobotics/ros_odrive)
    ├── odrive_ros2_control/         # ODrive ros2_control hardware interface plugin (vendored)
    └── odrive_velocity_pid/         # Cascaded PID controller node
```

- **`src/chrono_flap_sim/`** — Project Chrono inverted-pendulum flap simulation; runs as SIL plant or parallel shadow for model validation
- **`src/hil_odrive_ros2_control/`** — launch file (`motor_control.launch.py`), controller YAML (`config/controllers.yaml`), and URDF/Xacro (`description/urdf/motor.urdf.xacro`) for `motor_joint` (axis0) and `pto_joint` (axis1)
- **`src/odrive_velocity_pid/`** — velocity PID + feedforward node that reads `/joint_states`, generates a sinusoidal velocity reference, and publishes torque commands for Motor 1
- **`src/odrive_base/`** and **`src/odrive_ros2_control/`** — ODrive `ros2_control` hardware interface plugin and its base library, sourced from the upstream [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository

---

## Prerequisites

- **ROS 2 Jazzy** — source `/opt/ros/jazzy/setup.bash` in every terminal
- **ros2_control + ros2_controllers** — `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`
- **SocketCAN + can-utils** — `sudo apt-get install can-utils`
- **ODrive** configured for CAN communication (correct node IDs, CAN bitrate set to 250 kbps)

---

## CAN setup

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 250000
candump can0
```

> **Note:** bitrate must be **250000** — this matches the hardware configuration used here.

If `candump` shows no frames, check wiring/termination, verify the ODrive is powered, and confirm the bitrate matches what is configured in ODrive Tool.

---

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/hil_odrive_ros2_control   # root of this cloned repository
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package after making changes:

```bash
colcon build --packages-select odrive_velocity_pid
source install/setup.bash
```

---

## Launch the hardware stack

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hil_odrive_ros2_control motor_control.launch.py
```

This starts `ros2_control_node`, `robot_state_publisher`, `joint_state_broadcaster`, `motor_effort_controller`, and `pto_effort_controller`.

---

## Verify

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

`joint_state_broadcaster`, `motor_effort_controller`, and `pto_effort_controller` should all show as **active**. The `/joint_states` message should contain both `motor_joint` and `pto_joint` with valid (non-NaN) velocities.

> **Power telemetry:** The `electrical_power` and `mechanical_power` state interfaces are also available but will read NaN until you configure the ODrive to broadcast `Get_Powers` messages. See the [ODrive CAN broadcast setup](#pto-motor-configuration-phase-1--passive-linear-damper) section for the required `get_powers_msg_rate_ms` configuration step. Power values appear on the `/dynamic_joint_states` topic (not `/joint_states`):
> ```bash
> ros2 topic echo /dynamic_joint_states
> ```

---

## PTO motor configuration (Phase 1 — passive linear damper)

Configure ODrive axis1 as a passive linear damper directly via `odrivetool`. The velocity controller's P-gain acts as the damping coefficient `B`:

```python
# In odrivetool
odrv0.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_setpoint = 0
odrv0.axis1.controller.config.vel_gain = B   # damping coefficient (Nm·s/rad)
odrv0.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
```

When Motor 1 spins the shaft at angular velocity ω, axis1 applies resistive torque `τ = -B · ω`. Power is extracted and dissipated electrically.

**Power telemetry:** The hardware plugin exposes `electrical_power` and `mechanical_power` state interfaces read from ODrive `Get_Powers` CAN broadcast messages. Enable the broadcast in `odrivetool`:

```python
# In odrivetool — enable power telemetry broadcast on both axes (10 Hz)
odrv0.axis0.config.can.get_powers_msg_rate_ms = 100
odrv0.axis1.config.can.get_powers_msg_rate_ms = 100
odrv0.save_configuration()
```

Once configured, power values appear in `/dynamic_joint_states` — no oscilloscope needed.

---

## Run the velocity PID node (hydro emulator)

The node starts in `position_only` mode with a stationary trajectory (`amplitude = 0`, `omega = 0`).
Use `--ros-args` to configure it:

```bash
ros2 run odrive_velocity_pid velocity_pid_node --ros-args \
  -p control_mode:=cascade \
  -p amplitude_rad_s:=0.25 \
  -p omega_rad_s:=0.25
```

### Architecture note

`VelocityPidNode` is a **standalone ROS 2 node** — it is *not* a ros2_control controller plugin.
It subscribes to `/joint_states` for feedback and publishes directly to the effort controller topic.
This differs from the typical ros2_control pattern of writing a controller plugin, but allows the
PID node to be started, stopped, and tuned independently of the controller manager.

### Control modes

Three modes are available via the `control_mode` parameter (runtime-reconfigurable):

| Mode | Description |
|---|---|
| `position_only` *(default)* | Outer position PID output → torque directly. Good for commissioning. |
| `cascade` | Outer position PID → velocity command → inner velocity PID → torque. Recommended for full trajectory tracking. |
| `velocity_only` | Single velocity PID loop. Backward-compatible. |

Switch mode at runtime:

```bash
ros2 param set /velocity_pid_node control_mode cascade
```

### Tuning philosophy

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `kff` | `0.40` | Velocity feedforward — compensates viscous friction / back-EMF |
| `kaff` | `0.20` | Acceleration feedforward — compensates rotor inertia |
| `kp` | `0.35` | Proportional feedback for residual error correction |
| `ki` | `0.01` | Integral term removes steady-state drift. Directional anti-windup prevents lockup during saturation |
| `kd` | `0.0` | Not needed — feedforward handles dynamics, and CAN noise makes derivative unreliable |
| `filter_alpha` | `0.90` | 90% old + 10% new — aggressive smoothing to reject CAN velocity noise |
| `rate_hz` | `100.0` | 100 Hz control loop |
| `torque_limit_nm` | `0.40` | Safe torque limit with margin below the overcurrent threshold |
| `integral_limit` | `0.0` | Integrator disabled by default; set to a positive value to enable (e.g. `0.1`) |
| `invert_output` | `false` | Normal sign convention: positive torque → positive velocity |

---

## Set up visual feedback
When the system is running:

```bash
source install/setup.bash
ros2 run plotjuggler plotjuggler
```

This opens PlotJuggler where you can start streaming data. Useful topics to plot:
- `/velocity_pid_node/desired_velocity` and `/velocity_pid_node/measured_velocity` — velocity tracking
- `/velocity_pid_node/position_command` and `/velocity_pid_node/measured_position` — position tracking
- `/velocity_pid_node/velocity_error`, `/velocity_pid_node/position_error` — tracking errors
- `/velocity_pid_node/velocity_command` — outer-loop velocity command (cascade mode)

### PlotJuggler comparison: SIL and parallel modes

In **parallel mode**, compare the Chrono shadow prediction against real hardware measurements:

| Simulated (Chrono) | Real (hardware) |
|---|---|
| `/chrono_flap_node/sim_position` | `/velocity_pid_node/measured_position` |
| `/chrono_flap_node/sim_velocity` | `/velocity_pid_node/measured_velocity` |

If the traces track closely, the identified parameters (`flap_mass=0.5 kg`, `joint_stiffness=0.712441 N·m/rad`, `bearing_friction=0.2 N·m·s/rad`) are a good model of the plant. Divergence indicates where the model needs refinement.

In **SIL mode**, all kinematics come from the simulation:

| Topic | Description |
|---|---|
| `/chrono_flap_node/sim_position` | Simulated joint angle (rad) |
| `/chrono_flap_node/sim_velocity` | Simulated joint angular velocity (rad/s) |
| `/chrono_flap_node/sim_acceleration` | Simulated joint angular acceleration (rad/s²) |
| `/velocity_pid_node/measured_position` | Same as sim_position (PID reads from /joint_states → chrono) |
| `/velocity_pid_node/position_error` | Closed-loop position error (rad) |

```bash
source install/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

This opens `rqt_reconfigure`, where you can refresh the parameter list and select
`velocity_pid_node` or `chrono_flap_node` to access and change reconfigurable parameters at runtime.

---

## Data flow

### SIL mode (no hardware)

```
chrono_flap_node (sil_mode=true) → /joint_states → velocity_pid_node → /motor_effort_controller/commands → chrono_flap_node
```

`chrono_flap_node` publishes `/joint_states` at 100 Hz, replacing `joint_state_broadcaster`. `velocity_pid_node` is unaware of the source — the interface is identical.

### Parallel mode (with hardware)

```
ODrive HW → /joint_states → velocity_pid_node → /motor_effort_controller/commands → ODrive HW
                                                              │
                                                              ▼
                                                  chrono_flap_node (sil_mode=false)
                                                  ~/sim_position, ~/sim_velocity, ~/sim_acceleration

ODrive axis1 (pto_joint) ← passive damping configured via odrivetool (τ = -B·ω)

CAN → ODrive HW plugin → electrical_power, mechanical_power state interfaces → /dynamic_joint_states
```

`velocity_pid_node` is a **standalone node** — not a ros2_control controller plugin. It reads
`/joint_states` published by `joint_state_broadcaster` (hardware) or `chrono_flap_node` (SIL) and
writes directly to the effort controller's command topic.

---

## Full parameter reference

### Immutable (require node restart)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joint_state_topic` | `string` | `/joint_states` | Topic for `sensor_msgs/JointState` feedback |
| `command_topic` | `string` | `/motor_effort_controller/commands` | Topic for `std_msgs/Float64MultiArray` torque output |
| `joint_name` | `string` | `motor_joint` | Joint name inside `JointState.name[]` |
| `rate_hz` | `double` | `100.0` | Control loop rate (Hz) |

### Runtime-reconfigurable

#### Control mode & trajectory

| Parameter | Type | Default | Description |
|---|---|---|---|
| `control_mode` | `string` | `position_only` | Active mode: `position_only`, `cascade`, or `velocity_only` |
| `amplitude_rad_s` | `double` | `0.0` | Sine trajectory amplitude. In `velocity_only`: rad/s. In `cascade`/`position_only`: rad (peak excursion = `A/ω`). `0.0` = stationary. |
| `omega_rad_s` | `double` | `0.0` | Sine angular frequency (rad/s). For 1 Hz use `2π ≈ 6.283`. `0.0` = stationary. |
| `position_setpoint` | `double` | `0.0` | Static position setpoint (rad). Sine oscillates around this. |

#### Inner loop (velocity PID)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp` | `double` | `0.35` | Proportional gain |
| `ki` | `double` | `0.01` | Integral gain |
| `kd` | `double` | `0.0` | Derivative gain |
| `kff` | `double` | `0.40` | Velocity feedforward gain (suppressed when `kp = 0`) |
| `kaff` | `double` | `0.20` | Acceleration feedforward gain (suppressed when `kp = 0`) |
| `torque_limit_nm` | `double` | `0.40` | Output torque saturation limit (N·m) |
| `integral_limit` | `double` | `0.0` | Integral accumulator clamp. Must be positive to enable the integrator. |
| `deadband_rad_s` | `double` | `0.0` | Velocity error deadband — errors smaller than this are treated as zero |

#### Outer loop (position PID — `cascade` and `position_only` modes)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kp_pos` | `double` | `2.0` | Proportional gain |
| `ki_pos` | `double` | `0.01` | Integral gain |
| `kd_pos` | `double` | `0.025` | Derivative gain |
| `pos_integral_limit` | `double` | `1.0` | Outer-loop integral clamp (must be positive) |
| `pos_output_limit` | `double` | `2.0` | Maximum velocity command from outer loop (rad/s) |
| `outer_loop_divider` | `double` | `1.0` | Run outer loop every N inner-loop ticks (rate division) |

#### Miscellaneous

| Parameter | Type | Default | Description |
|---|---|---|---|
| `filter_alpha` | `double` | `0.90` | Velocity EMA smoothing coefficient. `0.0` = no filter, approaching `1.0` = heavier smoothing. Must be in `[0.0, 1.0)`. |
| `invert_output` | `bool` | `false` | Negate torque and flip signs of measured position/velocity |

---

## Tuning tips

1. **Start with low `torque_limit_nm`** (0.3-0.5 Nm) and low `omega_rad_s` (0.1-0.5) for safety
2. **Use `position_only` first** — tune `kp_pos`, `ki_pos`, `kd_pos` to hold a static setpoint cleanly before enabling `cascade`
3. **Tune `kff` and `kaff`** for the inner loop — they provide the bulk of the required torque without noise amplification. `kff ≈` viscous friction coefficient; `kaff ≈` rotor inertia J
4. **Keep `kp` moderate** (0.1-0.5) — CAN velocity noise (±5-10 rad/s) gets amplified by `kp`, causing oscillation at very high values
5. **Enable the integrator** by setting `integral_limit` to a positive value (e.g. `0.1`). The default `0.0` disables it; `ki` has no effect without an active integrator
6. **`filter_alpha=0.9`** (default) aggressively smooths CAN velocity noise — reduce toward `0.7` if the phase lag causes oscillation
7. **Motor sign convention** — if positive torque produces negative velocity, set `invert_output:=true`

---

## Identified plant parameters

Results from parameter identification on the 30 cm × 30 cm acrylic flap test bench:

| Property | Value | Method |
|---|---|---|
| Flap mass | 0.5 kg | Parameter identification |
| Joint stiffness (restoring spring) | 0.712441 N·m/rad | Parameter identification |
| Bearing friction (unpowered ODrive) | 0.2 N·m·s/rad | Parameter identification |
| Joint damping (powered ODrive) | 0.0 N·m·s/rad | Parameter identification |
| Flap dimensions | 0.30 × 0.30 × 0.0025 m | Measured |

These values are used as defaults in `chrono_flap_node` (see `src/chrono_flap_sim/src/chrono_flap_node.cpp`) and should be passed explicitly when running outside the launch file:

```bash
ros2 run chrono_flap_sim chrono_flap_node --ros-args \
  -p flap_mass_kg:=0.5 \
  -p joint_stiffness:=0.712441 \
  -p bearing_friction:=0.2
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `candump can0` shows nothing | CAN interface down or wrong bitrate | `sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 250000` |
| "Failed to send CAN frame" errors | CAN bus error state | Reset the interface (command above), check wiring/termination |
| Motor runaway | PID gains too high or wrong sign | Kill the node, set ODrive to IDLE (`odrivetool` → `odrv0.axis0.requested_state = 1`), clear errors |
| Motor not moving | `torque_limit_nm` too low to overcome friction | Increase `torque_limit_nm` |
| `/joint_states` has NaN velocity | Wrong ODrive node ID in URDF | Update `node_id` in `description/urdf/motor.urdf.xacro` |
| Controller type not found | `ros-jazzy-ros2-controllers` not installed | `sudo apt-get install ros-jazzy-ros2-controllers` |
| `pto_joint` missing from `/joint_states` | axis1 not calibrated/active on ODrive | Calibrate axis1 via `odrivetool` and set to `CLOSED_LOOP_CONTROL` |

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

## Sub-package documentation

- [`src/chrono_flap_sim/README.md`](src/chrono_flap_sim/README.md) — Project Chrono flap simulation: physics model, SIL vs parallel modes, all parameters, published topics, and build instructions
- [`src/hil_odrive_ros2_control/README.md`](src/hil_odrive_ros2_control/README.md) — hardware launch, URDF configuration, CAN node ID setup, and detailed controller bring-up steps
- [`src/odrive_velocity_pid/README.md`](src/odrive_velocity_pid/README.md) — cascaded PID node: control modes, all parameters, published topics, and safety features
- [`src/odrive_ros2_control/README.md`](src/odrive_ros2_control/README.md) — ODrive ros2_control hardware interface plugin (vendored from upstream)
- [`src/odrive_base/README.md`](src/odrive_base/README.md) — ODrive base library (vendored from upstream)

---

## Licensing

This repository's own code is original work. The `src/odrive_base/` and `src/odrive_ros2_control/` packages are sourced from the upstream [`odriverobotics/ros_odrive`](https://github.com/odriverobotics/ros_odrive) repository and retain its MIT license. Provenance details (upstream repository, commit SHA, and which packages were included) are documented in [`src/hil_odrive_ros2_control/VENDORED.md`](src/hil_odrive_ros2_control/VENDORED.md).
