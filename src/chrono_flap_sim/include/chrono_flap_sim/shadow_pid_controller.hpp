// shadow_pid_controller.hpp
//
// ShadowPidController — standalone header-only closed-loop PID controller for the Chrono
// flap simulation.  Replicates the exact control logic of VelocityPidNode so the simulation
// can close its own loop using sim_position / sim_velocity as feedback, making it inherently
// stable without a Luenberger observer.
//
// Three control modes are supported (identical to velocity_pid_node.cpp):
//
//   "position_only":
//     pos_pid → torque (clamped to torque_limit)
//
//   "cascade":
//     pos_pid → v_cmd (clamped to pos_output_limit) → vel_pid + kff + kaff → torque (clamped)
//
//   "velocity_only":
//     vel_pid + kff·vel_ref + kaff·accel_ref → torque (clamped)
//
// Trajectory (same formula as velocity_pid_node.cpp):
//   vel_ref   = amplitude * sin(omega * t)
//   accel_ref = amplitude * omega * cos(omega * t)
//   pos_ref   = position_setpoint + (amplitude / omega) * (1 - cos(omega * t))
//
// Usage:
//   ShadowPidController pid;
//   pid.control_mode      = "cascade";
//   pid.kp                = 0.35;
//   // ... set other gains ...
//   pid.reset();
//   double torque = pid.compute(sim_position, sim_velocity, t, dt);

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "odrive_velocity_pid/pid_controller.hpp"

class ShadowPidController
{
public:
  // ── Control mode ────────────────────────────────────────────────────────────────────────────────
  std::string control_mode{"position_only"};

  // ── Trajectory parameters ────────────────────────────────────────────────────────────────────
  double position_setpoint{0.0};   // rad — base position offset
  double amplitude_rad_s{0.0};     // rad/s — sine amplitude (velocity amplitude)
  double omega_rad_s{0.0};         // rad/s — sine angular frequency

  // ── Inner loop (velocity PID) ────────────────────────────────────────────────────────────────
  double kp{0.35};
  double ki{0.01};
  double kd{0.0};
  double kff{0.40};                // velocity feedforward gain
  double kaff{0.20};               // acceleration feedforward gain
  double torque_limit_nm{0.40};    // symmetric torque clamp (N·m)
  double integral_limit{0.0};      // integral accumulator clamp (0 = unlimited)
  double deadband_rad_s{0.0};      // velocity error deadband (rad/s)

  // ── Outer loop (position PID) ────────────────────────────────────────────────────────────────
  double kp_pos{2.00};
  double ki_pos{0.01};
  double kd_pos{0.025};
  double pos_integral_limit{1.0};  // outer integral accumulator clamp
  double pos_output_limit{2.0};    // symmetric velocity command clamp (rad/s)

  // ── Outer loop rate divider ──────────────────────────────────────────────────────────────────
  double outer_loop_divider{1.0};  // run outer loop every N inner ticks

  // ── Velocity filter ─────────────────────────────────────────────────────────────────────────
  double filter_alpha{0.90};       // IIR filter coefficient in [0.0, 1.0)

  // ── reset — clear all state ──────────────────────────────────────────────────────────────────
  void reset()
  {
    vel_pid_.reset();
    pos_pid_.reset();
    filtered_vel_         = 0.0;
    filter_initialized_   = false;
    outer_loop_counter_   = 0;
    outer_dt_accum_       = 0.0;
    v_cmd_                = 0.0;
  }

  // ── compute — one control tick ───────────────────────────────────────────────────────────────
  //
  // Parameters:
  //   sim_position  – current simulated joint angle (rad)
  //   sim_velocity  – current simulated joint angular velocity (rad/s)
  //   t             – elapsed time since start (s); used for trajectory generation
  //   dt            – time since last call (s); must be > 0
  //
  // Returns the torque command (N·m), clamped to ±torque_limit_nm.
  double compute(double sim_position, double sim_velocity, double t, double dt)
  {
    apply_gains();

    // ── Velocity filter ────────────────────────────────────────────────────────────────────────
    if (!filter_initialized_) {
      filtered_vel_       = sim_velocity;
      filter_initialized_ = true;
    } else {
      filtered_vel_ = filter_alpha * filtered_vel_ + (1.0 - filter_alpha) * sim_velocity;
    }

    // ── Trajectory ────────────────────────────────────────────────────────────────────────────
    double vel_ref   = 0.0;
    double accel_ref = 0.0;
    double pos_ref   = position_setpoint;
    if (std::abs(omega_rad_s) > 1e-9) {
      vel_ref   = amplitude_rad_s * std::sin(omega_rad_s * t);
      accel_ref = amplitude_rad_s * omega_rad_s * std::cos(omega_rad_s * t);
      if (control_mode != "velocity_only") {
        pos_ref = position_setpoint + (amplitude_rad_s / omega_rad_s) * (1.0 - std::cos(omega_rad_s * t));
      }
    }

    // ── Mode dispatch ─────────────────────────────────────────────────────────────────────────
    double torque = 0.0;
    if (control_mode == "cascade") {
      run_outer_loop(pos_ref, vel_ref, sim_position, dt);
      torque = run_inner_loop(v_cmd_, accel_ref, dt);
    } else if (control_mode == "position_only") {
      torque = pos_pid_.compute(pos_ref, sim_position, dt);
      torque = std::clamp(torque, -torque_limit_nm, torque_limit_nm);
      pos_pid_.saturated = (std::abs(torque) >= torque_limit_nm);
      v_cmd_ = 0.0;
    } else {  // velocity_only
      v_cmd_ = vel_ref;
      torque = run_inner_loop(vel_ref, accel_ref, dt);
    }

    return torque;
  }

private:
  // ── PID controllers ──────────────────────────────────────────────────────────────────────────
  PidController vel_pid_;
  PidController pos_pid_;

  // ── Internal state ───────────────────────────────────────────────────────────────────────────
  double filtered_vel_{0.0};
  bool   filter_initialized_{false};
  int    outer_loop_counter_{0};
  double outer_dt_accum_{0.0};
  double v_cmd_{0.0};

  // ── apply_gains — sync PID structs from public member fields ─────────────────────────────────
  // Called at the start of each compute() so changes to gains take effect immediately.
  void apply_gains()
  {
    vel_pid_.kp             = kp;
    vel_pid_.ki             = ki;
    vel_pid_.kd             = kd;
    vel_pid_.integral_limit = integral_limit;
    vel_pid_.deadband       = deadband_rad_s;

    pos_pid_.kp             = kp_pos;
    pos_pid_.ki             = ki_pos;
    pos_pid_.kd             = kd_pos;
    pos_pid_.integral_limit = pos_integral_limit;
  }

  // ── run_inner_loop — velocity PID + feedforward → clamped torque ────────────────────────────
  double run_inner_loop(double vel_setpoint, double accel_ff, double dt)
  {
    double pid_out = vel_pid_.compute(vel_setpoint, filtered_vel_, dt);

    // Feedforward is only safe when kp > 0 provides damping feedback.
    // Without feedback, kff/kaff inject open-loop torque that can cause runaway.
    double ff = 0.0;
    if (kp > 0.0) {
      ff = kff * vel_setpoint + kaff * accel_ff;
    }

    double torque = pid_out + ff;
    torque = std::clamp(torque, -torque_limit_nm, torque_limit_nm);
    vel_pid_.saturated = (std::abs(torque) >= torque_limit_nm);
    return torque;
  }

  // ── run_outer_loop — position PID → v_cmd_, rate-divided ────────────────────────────────────
  void run_outer_loop(double pos_ref, double vel_ff, double sim_position, double dt)
  {
    outer_dt_accum_ += dt;
    if (++outer_loop_counter_ >= static_cast<int>(std::llround(outer_loop_divider))) {
      outer_loop_counter_ = 0;
      double pid_out = pos_pid_.compute(pos_ref, sim_position, outer_dt_accum_);
      v_cmd_ = std::clamp(pid_out + vel_ff, -pos_output_limit, pos_output_limit);
      pos_pid_.saturated = (std::abs(v_cmd_) >= pos_output_limit);
      outer_dt_accum_ = 0.0;
    }
  }
};
