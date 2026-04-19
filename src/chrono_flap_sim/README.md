# chrono_flap_sim

ROS 2 C++ package that runs a **Project Chrono** multibody simulation of a motor driving a rigid
flap about a revolute joint. The node subscribes to the same effort/torque topic that
`velocity_pid_node` publishes to, steps the Chrono simulation at a configurable rate, and
publishes simulated kinematics so you can compare **desired**, **motor**, and **simulated**
kinematics side-by-side in PlotJuggler.

## Published topics

All topics are namespaced under `~/` (i.e. `/<node_name>/`):

| Topic | Type | Description |
|---|---|---|
| `~/sim_position` | `std_msgs/Float64` | Simulated joint angle (rad) |
| `~/sim_velocity` | `std_msgs/Float64` | Simulated joint angular velocity (rad/s) |
| `~/sim_acceleration` | `std_msgs/Float64` | Simulated joint angular acceleration (rad/s²) |

## Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/motor_effort_controller/commands` | `std_msgs/Float64MultiArray` | Effort/torque command (first element used) |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `amplitude_rad_s` | double | `0.0` | Sine trajectory amplitude (mirrors `velocity_pid_node`) |
| `omega_rad_s` | double | `0.0` | Sine trajectory angular frequency (mirrors `velocity_pid_node`) |
| `rate_hz` | double | `100.0` | Simulation step rate (Hz) |
| `flap_length_m` | double | `0.3` | Flap length (m) |
| `flap_mass_kg` | double | `0.05` | Flap mass (kg) |
| `joint_damping` | double | `0.001` | Revolute joint viscous damping coefficient (N·m·s/rad) |
| `effort_topic` | string | `/motor_effort_controller/commands` | Topic to subscribe for torque |

## Prerequisites

* [Project Chrono](https://projectchrono.org) — core library only (`ChronoEngine`).
* ROS 2 Jazzy (or compatible).

## Building

```bash
# Make sure Chrono is installed and ChronoConfig.cmake is findable.
# If it is not on CMake's default path, pass -DChronoConfig_DIR:
colcon build --packages-select chrono_flap_sim \
  --cmake-args -DChronoConfig_DIR=/path/to/chrono/lib/cmake/Chrono
```

## Usage

The node is launched automatically via `motor_control.launch.py`.  To run it standalone:

```bash
ros2 run chrono_flap_sim chrono_flap_node --ros-args \
  -p rate_hz:=100.0 \
  -p flap_length_m:=0.3 \
  -p flap_mass_kg:=0.05 \
  -p joint_damping:=0.001
```

## PlotJuggler comparison

After launching the full stack, subscribe to the following topics in PlotJuggler:

* `velocity_pid_node/desired_velocity` — desired kinematics from the PID node
* `/joint_states` — measured kinematics from the motor hardware
* `chrono_flap_node/sim_velocity` — simulated kinematics from this node
