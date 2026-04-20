# Parallel mode: Luenberger observer correction

## The open-loop drift problem

In **parallel mode** (`sil_mode=false`), `chrono_flap_node` runs as a pure open-loop shadow
of the real hardware. Both the real ODrive and Project Chrono receive the same torque command
and independently integrate their own equations of motion — but there is zero coupling between
their states:

```
PID → torque → Real ODrive → real position → PID  (closed-loop)
                    │
                    └─ (same torque, copy)
                         ↓
                      Chrono → sim position  (open-loop, nobody reads this)
```

Even though the velocity model may be accurate enough that instantaneous velocities match,
any tiny velocity error accumulates in the position integral over time:

```
position_error(t) = ∫₀ᵗ (v_real − v_sim) dτ
```

This integral grows without bound, causing:

- **DC offset** for step/position setpoints (constant velocity bias integrates to a ramp)
- **Amplitude drift and phase error** for sine-wave inputs (even a 1% velocity bias at 1 rad/s
  produces ~0.01 rad/s of position drift per second)

RVIZ2 looks fine because it reads from the real hardware's `/joint_states`, not from the sim.

## Why SIL mode does not have this problem

In **SIL mode** (`sil_mode=true`), there is only **one integrator** — the Chrono simulation
is the plant. The PID controller reads its own output back via `/joint_states` and actively
corrects any position error:

```
PID → torque → Chrono → position/velocity → PID  (single closed loop)
```

There is no second "truth" to drift away from. The same model mismatch that causes drift in
parallel mode is irrelevant in SIL mode because the PID drives the error to zero — there is
no "real vs sim" discrepancy possible when the sim is the only reality.

## The Luenberger observer correction

The standard solution is to weakly couple the simulated state back to the measured state. This
is a **Luenberger observer** (also called a state observer or prediction-correction observer).

Each tick, after Chrono integrates, the predicted position is nudged toward the measured position:

```
θ_corrected = θ_predicted + α · (θ_measured − θ_predicted)
```

where:
- `θ_predicted` is the angle output by `motor_link_->GetMotorAngle()` after `DoStepDynamics`
- `θ_measured` is the angle from the real hardware's `/joint_states` message
- `α` is the `observer_gain` parameter (default `0.05`)
- `θ_corrected` is written back into the Chrono body state for the next integration step

The corrected state is also fed back into the Chrono rigid body (via `flap_->SetRot()` and
`flap_->SetPos()`) so that the next `DoStepDynamics` call begins from the corrected angle
rather than re-predicting from the uncorrected one.

## Frequency-domain interpretation

The correction term `α · (θ_measured − θ_predicted)` acts as a **low-pass filter on the
innovation** (the difference between measurement and prediction). It introduces a correction
bandwidth:

```
f_correction ≈ α / (2π · dt)
```

At `rate_hz = 100 Hz` (`dt = 0.01 s`) and `α = 0.05`:

```
f_correction ≈ 0.05 / (2π · 0.01) ≈ 0.8 Hz
```

This means:
- **Dynamics faster than ~0.8 Hz** → Chrono governs. The observer barely reacts before the
  error has already changed sign. You see the real physics model in the `~/sim_*` topics.
- **Errors slower than ~0.8 Hz** (i.e. slow drift) → measurement governs. The observer
  steadily corrects the accumulated error.

The crossover frequency scales linearly with `α`: increasing `α` raises the bandwidth,
meaning you trust the measurement more at higher frequencies. Decreasing `α` lowers the
bandwidth, meaning Chrono runs more independently.

## Physical analogy: weak spring between sim and reality

Think of the correction term as attaching a **weak spring** between the simulated position
and the measured (real) position:

```
τ_correction = α · (θ_measured − θ_sim)
```

- **Stiff spring** (large α): The sim is dragged tightly to reality each tick. Accurate
  position but you cannot see independent sim dynamics anymore.
- **Weak spring** (small α): The sim floats mostly freely with a gentle tug. Keeps its own
  transient character but does not wander far from reality.
- **No spring** (α = 0): The current open-loop behavior. The sim floats freely and eventually
  drifts away.

## Error decay

The correction is applied once per tick. The tracking error after `n` ticks decays as:

```
error(n) = error(0) · (1 − α)^n
```

At `α = 0.05` and `rate_hz = 100 Hz`:

| Elapsed time | Ticks `n` | Remaining fraction |
|---|---|---|
| 0.1 s | 10 | (0.95)^10 ≈ 0.60 |
| 0.5 s | 50 | (0.95)^50 ≈ 0.08 |
| 1.0 s | 100 | (0.95)^100 ≈ 0.006 |
| 2.0 s | 200 | (0.95)^200 ≈ 3.5×10⁻⁵ |

The time constant (time to reach 1/e ≈ 37% of the initial error) is:

```
τ = −dt / ln(1 − α) ≈ dt / α   (for small α)
```

At `dt = 0.01 s` and `α = 0.05`: `τ ≈ 0.2 s`. Drift accumulated over any time scale longer
than ~0.2 s is effectively suppressed.

## Suggested α values

| `observer_gain` α | Time constant τ (at 100 Hz) | Effect |
|---|---|---|
| `0.0` | ∞ | No correction — fully open-loop. Position will drift (current behavior without observer). |
| `0.01` | ~1.0 s | Very gentle correction. Drift corrects over ~1 s. Minimal impact on transients. |
| `0.05` *(default)* | ~0.2 s | Good default. Drift gone within ~0.2 s. Transients well preserved. |
| `0.10` | ~0.1 s | Aggressive. Starts smoothing out short-duration transients. |
| `1.0` | 0 s (one-shot) | Sim snaps to measurement every tick. No independent Chrono dynamics visible. |

## Relationship to Kalman filtering

The Luenberger observer correction is structurally identical to the **update step of a Kalman
filter**:

```
x̂(k|k) = x̂(k|k−1) + K · (y(k) − H · x̂(k|k−1))
```

where `x̂(k|k−1)` is `θ_predicted`, `y(k)` is `θ_measured`, `H = 1` (direct angle
measurement), and `K` is the Kalman gain.

In a Kalman filter, `K` is computed optimally from the process noise covariance (model
uncertainty) and measurement noise covariance. Here, `α` plays the same role but is
hand-tuned rather than derived from noise statistics. This is appropriate because:

1. The model is already good (velocities match), so the main uncertainty is the slow
   position drift — not random noise.
2. The measurement noise on `/joint_states` is low (ODrive encoder resolution is high).
3. A hand-tuned constant is simpler to reason about and adjust.

If you want to go further, you could implement a full discrete Kalman filter with separate
`Q` (process noise) and `R` (measurement noise) matrices, but for this application the
constant-gain observer is sufficient.

## Practical tuning advice

1. **Start at α = 0.05** (the default). This is conservative — it kills slow drift without
   disturbing any dynamics above ~1 Hz.

2. **Plot sim vs real in PlotJuggler**:
   - `/chrono_flap_node/sim_position` vs `/velocity_pid_node/measured_position`
   - `/chrono_flap_node/sim_velocity` vs `/velocity_pid_node/measured_velocity`

3. **If position still drifts** (traces diverge slowly over seconds): increase `α` toward
   0.1. The correction bandwidth is too low for the drift rate.

4. **If transient dynamics look like a smoothed copy of the measurement** rather than
   independent Chrono physics: decrease `α` toward 0.01–0.02. The bandwidth is too high
   and the observer is dominating the fast response.

5. **Set `observer_gain:=0.0`** temporarily to confirm you are seeing the same drift as
   before. This rules out other causes (e.g. wrong joint name, stale measurement).

6. The `observer_gain` parameter is **dynamically reconfigurable** via `rqt_reconfigure`
   — you can tune it live without restarting the node.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
# Select chrono_flap_node → adjust observer_gain slider
```
