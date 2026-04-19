// velocity_pid_node.cpp
//
// VelocityPidNode — ROS 2 node that runs a cascaded PID controller for an ODrive motor.
//
// Architecture overview:
//   Three operating modes are selected by the `control_mode` parameter:
//
//   "position_only":
//     Outer position PID drives the torque directly.  Useful for checking position gains before
//     enabling the full cascade.  The sine trajectory produces a position reference; the position
//     PID output is saturated and sent directly as torque.
//
//   "cascade" (default, recommended for full trajectory tracking):
//     Outer loop (position PID) → velocity command → inner loop (velocity PID) → torque.
//     The sine trajectory produces a position reference with analytical velocity/accel derivatives
//     used as feedforwards at each level.  Proper bandwidth separation greatly improves stability.
//
//   "velocity_only" (backward-compatible):
//     Single velocity-tracking loop.  The sine trajectory produces a velocity setpoint directly.
//     Equivalent to the original flat-PID behaviour.
//
// PID controller:
//   Both loops use the same generic PidController class (pid_controller.hpp) which encapsulates
//   anti-windup, integral clamping, derivative-on-measurement, and output saturation.
//
// Parameter infrastructure:
//   Numeric / bool parameters are declared through the param_defs_ table (one entry → full
//   declare/read/validate/apply lifecycle handled automatically).  The string parameter
//   `control_mode` is handled manually in the validate / apply callbacks because std::variant
//   only covers double and bool.
//
// Safety:
//   - Inner loop always enforces torque saturation.
//   - On shutdown, zero torque is published.
//   - If the joint is not found, zero torque is published every tick.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <variant>
#include <vector>

#include "odrive_velocity_pid/pid_controller.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VelocityPidNode : public rclcpp::Node
{
public:
  VelocityPidNode()
  : Node("velocity_pid_node"),
    last_measured_vel_(0.0),
    filtered_vel_(0.0),
    filter_initialized_(false),
    joint_index_(-1),
    last_measured_pos_(0.0),
    outer_loop_counter_(0),
    outer_dt_accum_(0.0),
    v_cmd_(0.0)
  {
    // ── String parameters (not runtime-reconfigurable) ────────────────────────────────────────
    // joint_state_topic, command_topic, joint_name require re-creating subscribers/publishers
    // so they are intentionally excluded from the param_defs_ table.
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("command_topic", "/motor_effort_controller/commands");
    this->declare_parameter<std::string>("joint_name", "motor_joint");

    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    command_topic_     = this->get_parameter("command_topic").as_string();
    joint_name_        = this->get_parameter("joint_name").as_string();

    // ── control_mode string (runtime-reconfigurable, handled separately) ─────────────────────
    this->declare_parameter<std::string>("control_mode", "cascade");
    control_mode_ = this->get_parameter("control_mode").as_string();
    if (!is_valid_control_mode(control_mode_)) {
      RCLCPP_WARN(this->get_logger(),
        "Invalid control_mode '%s'; falling back to 'position_only'", control_mode_.c_str());
      control_mode_ = "position_only";
    }

    // ── Build the parameter definition table ─────────────────────────────────────────────────
    // Each entry drives the full declare / read / validate / apply / reset lifecycle
    // for one tunable numeric or bool parameter.  Adding a parameter requires one line here.
    //
    // Fields:
    //   name         – ROS parameter name
    //   default_value – default (also encodes the C++ type via the variant alternative)
    //   member       – pointer-to-member for direct assignment in loops
    //   validator    – optional ValidatorFn; empty = always valid
    //   reset_flags  – bitmask: kResetVel (1) resets inner PID, kResetPos (2) resets outer PID
    param_defs_ = {
      // ── Trajectory ─────────────────────────────────────────────────────────────────────────
      {"amplitude_rad_s",        0.00,   &VelocityPidNode::amplitude_rad_s_,     {},                                      0},
      {"omega_rad_s",            0.00,   &VelocityPidNode::omega_rad_s_,         {},                                      0},
      // ── Inner loop (velocity PID) gains ────────────────────────────────────────────────────
      {"kp",                     0.35,   &VelocityPidNode::kp_,                  {},                                      kResetVel},
      {"ki",                     0.01,   &VelocityPidNode::ki_,                  {},                                      kResetVel},
      {"kd",                     0.00,   &VelocityPidNode::kd_,                  {},                                      kResetVel},
      {"kff",                    0.40,   &VelocityPidNode::kff_,                 {},                                      kResetVel},
      {"kaff",                   0.00,   &VelocityPidNode::kaff_,                {},                                      kResetVel},
      // ── Inner loop limits ──────────────────────────────────────────────────────────────────
      {"torque_limit_nm",        0.40,   &VelocityPidNode::torque_limit_nm_,     positive_validator("torque_limit_nm"),   0},
      {"integral_limit",         0.00,   &VelocityPidNode::integral_limit_,      positive_validator("integral_limit"),    0},
      {"deadband_rad_s",         0.00,   &VelocityPidNode::deadband_rad_s_,      {},                                      0},
      // ── Outer loop (position PID) gains ────────────────────────────────────────────────────
      {"kp_pos",                 2.00,   &VelocityPidNode::kp_pos_,              {},                                      kResetPos},
      {"ki_pos",                 0.01,   &VelocityPidNode::ki_pos_,              {},                                      kResetPos},
      {"kd_pos",                0.025,   &VelocityPidNode::kd_pos_,              {},                                      kResetPos},
      // ── Outer loop limits ──────────────────────────────────────────────────────────────────
      {"pos_integral_limit",     1.00,   &VelocityPidNode::pos_integral_limit_,  positive_validator("pos_integral_limit"), 0},
      {"pos_output_limit",       2.00,   &VelocityPidNode::pos_output_limit_,    positive_validator("pos_output_limit"),  0},
      // ── Outer loop rate divider ─────────────────────────────────────────────────────────────
      {"outer_loop_divider",     1.00,   &VelocityPidNode::outer_loop_divider_,  positive_validator("outer_loop_divider"), 0},
      // ── Misc ───────────────────────────────────────────────────────────────────────────────
      {"rate_hz",              100.0,    &VelocityPidNode::rate_hz_,             positive_validator("rate_hz"),           0},
      {"filter_alpha",           0.90,   &VelocityPidNode::filter_alpha_,        unit_range_validator("filter_alpha"),    0},
      {"invert_output",         false,   &VelocityPidNode::invert_output_,       {},                                      0},
      {"position_setpoint",      0.00,   &VelocityPidNode::position_setpoint_,   {},                                      0},
    };

    // ── Declare, read, and startup-validate each table parameter ─────────────────────────────
    for (auto & def : param_defs_) {
      std::visit([&](auto default_val) {
        using T = decltype(default_val);
        this->declare_parameter<T>(def.name, default_val);

        auto & member = std::get<T VelocityPidNode::*>(def.member);
        if constexpr (std::is_same_v<T, double>) {
          this->*member = this->get_parameter(def.name).as_double();
        } else {
          this->*member = this->get_parameter(def.name).as_bool();
        }
      }, def.default_value);

      if (def.validator) {
        auto vr = def.validator(get_member_value(def));
        if (!vr.successful) {
          RCLCPP_WARN(this->get_logger(), "%s; using default", vr.reason.c_str());
          std::visit([&](auto default_val) {
            using T = decltype(default_val);
            auto & member = std::get<T VelocityPidNode::*>(def.member);
            this->*member = default_val;
          }, def.default_value);
        }
      }
    }

    // Nominal timer period derived from the validated rate.
    dt_ = 1.0 / rate_hz_;
    last_loop_time_ = this->now();

    // ── Initialise PID controllers from the loaded parameters ────────────────────────────────
    apply_vel_pid_config();
    apply_pos_pid_config();

    // ── Subscriber ───────────────────────────────────────────────────────────────────────────
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_state_callback(msg);
      });

    // ── Publishers ───────────────────────────────────────────────────────────────────────────
    torque_pub_            = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    desired_vel_pub_       = this->create_publisher<std_msgs::msg::Float64>("~/desired_velocity", 10);
    measured_vel_pub_      = this->create_publisher<std_msgs::msg::Float64>("~/measured_velocity", 10);
    velocity_error_pub_    = this->create_publisher<std_msgs::msg::Float64>("~/velocity_error", 10);
    measured_position_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/measured_position", 10);
    position_error_pub_    = this->create_publisher<std_msgs::msg::Float64>("~/position_error", 10);
    position_command_pub_  = this->create_publisher<std_msgs::msg::Float64>("~/position_command", 10);
    velocity_command_pub_  = this->create_publisher<std_msgs::msg::Float64>("~/velocity_command", 10);

    // ── Control loop timer ───────────────────────────────────────────────────────────────────
    const auto timer_period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      [this]() { control_loop(); });

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "VelocityPidNode started: joint='%s', rate=%.1f Hz, mode='%s', "
      "vel_pid kp=%.3f ki=%.3f kd=%.3f  pos_pid kp=%.3f ki=%.3f kd=%.3f",
      joint_name_.c_str(), rate_hz_, control_mode_.c_str(),
      kp_, ki_, kd_, kp_pos_, ki_pos_, kd_pos_);

    // ── Register parameter callbacks ─────────────────────────────────────────────────────────
    on_set_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_validate_parameters(params);
      });
    post_set_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        on_apply_parameters(params);
      });
  }

  // ── Destructor ───────────────────────────────────────────────────────────────────────────────
  ~VelocityPidNode()
  {
    if (timer_) {
      timer_->cancel();
    }
    if (torque_pub_) {
      publish_torque(0.0);
      RCLCPP_INFO(this->get_logger(), "Shutdown: 0 torque");
    }
  }

private:
  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Parameter table infrastructure
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  using MemberPtr   = std::variant<double VelocityPidNode::*, bool VelocityPidNode::*>;
  using ParamValue  = std::variant<double, bool>;
  using ValidatorFn = std::function<rcl_interfaces::msg::SetParametersResult(const ParamValue &)>;

  static constexpr uint8_t kResetVel = 1;  // bitmask: reset inner (velocity) PID integrator
  static constexpr uint8_t kResetPos = 2;  // bitmask: reset outer (position) PID integrator

  struct ParamDef
  {
    std::string  name;
    ParamValue   default_value;
    MemberPtr    member;
    ValidatorFn  validator;
    uint8_t      reset_flags = 0;  // kResetVel | kResetPos as needed
  };

  // ── Validator factories ───────────────────────────────────────────────────────────────────────

  static ValidatorFn positive_validator(const std::string & param_name)
  {
    return [param_name](const ParamValue & v) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = (std::get<double>(v) > 0.0);
      if (!r.successful) {
        r.reason = param_name + " must be positive.";
      }
      return r;
    };
  }

  static ValidatorFn unit_range_validator(const std::string & param_name)
  {
    return [param_name](const ParamValue & v) {
      rcl_interfaces::msg::SetParametersResult r;
      const double val = std::get<double>(v);
      r.successful = (val >= 0.0 && val < 1.0);
      if (!r.successful) {
        r.reason = param_name + " must be in [0.0, 1.0).";
      }
      return r;
    };
  }

  ParamValue get_member_value(const ParamDef & def) const
  {
    return std::visit([this](auto member_ptr) -> ParamValue {
      return this->*member_ptr;
    }, def.member);
  }

  // ── Immutable parameters ──────────────────────────────────────────────────────────────────────
  static inline const std::set<std::string> kImmutableParams = {
    "joint_state_topic", "command_topic", "joint_name", "rate_hz"
  };

  // ── Valid control modes ───────────────────────────────────────────────────────────────────────
  static bool is_valid_control_mode(const std::string & mode)
  {
    return mode == "velocity_only" || mode == "cascade" || mode == "position_only";
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  on_validate_parameters — validation only, no side-effects
  // ══════════════════════════════════════════════════════════════════════════════════════════════
  rcl_interfaces::msg::SetParametersResult on_validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      // Reject immutable params.
      if (kImmutableParams.count(param.get_name())) {
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() +
          "' cannot be changed at runtime (requires node restart).";
        return result;
      }

      // Validate control_mode string.
      if (param.get_name() == "control_mode") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "control_mode must be a string.";
          return result;
        }
        if (!is_valid_control_mode(param.as_string())) {
          result.successful = false;
          result.reason = "control_mode must be 'cascade', 'velocity_only', or 'position_only'.";
          return result;
        }
      }

      // Validate table params.
      for (const auto & def : param_defs_) {
        if (def.name == param.get_name() && def.validator) {
          const ParamValue incoming =
            (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            ? ParamValue{param.as_double()}
            : ParamValue{param.as_bool()};
          auto vr = def.validator(incoming);
          if (!vr.successful) {
            return vr;
          }
        }
      }
    }

    return result;
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  on_apply_parameters — mutate member variables (called after successful validation)
  // ══════════════════════════════════════════════════════════════════════════════════════════════
  void on_apply_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool vel_gains_changed = false;
    bool pos_gains_changed = false;

    for (const auto & param : parameters) {
      // Handle the runtime-reconfigurable string parameter control_mode.
      if (param.get_name() == "control_mode") {
        control_mode_ = param.as_string();
        // Reset outer loop counter and accumulated time so the new mode starts cleanly.
        outer_loop_counter_ = 0;
        outer_dt_accum_     = 0.0;
        RCLCPP_INFO(this->get_logger(), "Parameter updated: control_mode = '%s'",
          control_mode_.c_str());
        continue;
      }

      // Handle table params.
      for (auto & def : param_defs_) {
        if (def.name != param.get_name()) {
          continue;
        }

        std::visit([&](auto member_ptr) {
          using T = std::remove_reference_t<decltype(this->*member_ptr)>;
          if constexpr (std::is_same_v<T, double>) {
            this->*member_ptr = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %.4f",
              def.name.c_str(), param.as_double());
          } else {
            this->*member_ptr = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Parameter updated: %s = %s",
              def.name.c_str(), param.as_bool() ? "true" : "false");
          }
        }, def.member);

        if (def.reset_flags & kResetVel) { vel_gains_changed = true; }
        if (def.reset_flags & kResetPos) { pos_gains_changed = true; }
        break;
      }
    }

    // Sync gains and reset integrators as needed.
    if (vel_gains_changed) {
      vel_pid_.reset();
      RCLCPP_INFO(this->get_logger(),
        "Velocity PID gains changed — integrator reset. New gains: kp=%.4f ki=%.4f kd=%.4f",
        kp_, ki_, kd_);
    }
    apply_vel_pid_config();

    if (pos_gains_changed) {
      pos_pid_.reset();
      RCLCPP_INFO(this->get_logger(),
        "Position PID gains changed — integrator reset. New gains: kp=%.4f ki=%.4f kd=%.4f",
        kp_pos_, ki_pos_, kd_pos_);
    }
    apply_pos_pid_config();

    // Reset outer loop counter if divider changed.
    for (const auto & param : parameters) {
      if (param.get_name() == "outer_loop_divider") {
        outer_loop_counter_ = 0;
        outer_dt_accum_     = 0.0;
        break;
      }
    }
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  PID configuration helpers
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  // apply_vel_pid_config — sync the inner-loop (velocity) PID gains and limits from member vars.
  // Called after the constructor param load and after any param update that touches these fields.
  void apply_vel_pid_config()
  {
    vel_pid_.kp             = kp_;
    vel_pid_.ki             = ki_;
    vel_pid_.kd             = kd_;
    vel_pid_.integral_limit = integral_limit_;
    vel_pid_.deadband       = deadband_rad_s_;
    // Output saturation is enforced externally (after adding feedforward) so output_min/max
    // are left at their default ±1e9 (no internal PID clamping).
  }

  // apply_pos_pid_config — sync the outer-loop (position) PID gains and limits from member vars.
  void apply_pos_pid_config()
  {
    pos_pid_.kp             = kp_pos_;
    pos_pid_.ki             = ki_pos_;
    pos_pid_.kd             = kd_pos_;
    pos_pid_.integral_limit = pos_integral_limit_;
    // Output saturation is enforced externally after adding velocity feedforward.
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Joint-state callback
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (joint_index_ < 0 ||
        static_cast<size_t>(joint_index_) >= msg->name.size() ||
        msg->name[static_cast<size_t>(joint_index_)] != joint_name_)
    {
      joint_index_ = -1;
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint_name_) {
          joint_index_ = static_cast<int>(i);
          break;
        }
      }
      if (joint_index_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Joint '%s' not found in /joint_states", joint_name_.c_str());
        return;
      }
    }

    const size_t idx = static_cast<size_t>(joint_index_);
    if (idx < msg->velocity.size()) {
      double raw = msg->velocity[idx];
      if (invert_output_) { raw = -raw; }

      if (!filter_initialized_) {
        filtered_vel_       = raw;
        filter_initialized_ = true;
      } else {
        filtered_vel_ = filter_alpha_ * filtered_vel_ + (1.0 - filter_alpha_) * raw;
      }
      last_measured_vel_ = filtered_vel_;
    }

    if (idx < msg->position.size()) {
      double raw_pos = msg->position[idx];
      if (invert_output_) { raw_pos = -raw_pos; }
      last_measured_pos_ = raw_pos;
    }
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Helper: publish a Float64 message
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  void pub_f64(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double val)
  {
    std_msgs::msg::Float64 m;
    m.data = val;
    pub->publish(m);
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Trajectory reference
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  struct TrajectoryRef { double pos, vel, accel; };

  TrajectoryRef compute_trajectory(double t) const
  {
    TrajectoryRef ref{position_setpoint_, 0.0, 0.0};
    if (std::abs(omega_rad_s_) > 1e-9) {
      ref.vel   = amplitude_rad_s_ * std::sin(omega_rad_s_ * t);
      ref.accel = amplitude_rad_s_ * omega_rad_s_ * std::cos(omega_rad_s_ * t);
      if (control_mode_ != "velocity_only") {
        ref.pos = position_setpoint_ +
          (amplitude_rad_s_ / omega_rad_s_) * (1.0 - std::cos(omega_rad_s_ * t));
      }
    }
    return ref;
  }

  // ── run_inner_loop — clamped torque from velocity PID + feedforward ────────────────────────────
  double run_inner_loop(double vel_setpoint, double accel_ff, double dt)
  {
    double pid_out = vel_pid_.compute(vel_setpoint, last_measured_vel_, dt);
    // Feedforward is only safe when kp_ > 0 provides damping feedback.
    // Without feedback, kff/kaff inject open-loop torque that causes runaway.
    double ff = 0.0;
    if (kp_ > 0.0) {
      ff = kff_ * vel_setpoint + kaff_ * accel_ff;
    } else if (kff_ != 0.0 || kaff_ != 0.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Feedforward gains (kff=%.3f, kaff=%.3f) are nonzero but kp=0; "
        "feedforward suppressed to prevent open-loop runaway.",
        kff_, kaff_);
    }
    double torque = pid_out + ff;
    torque         = std::clamp(torque, -torque_limit_nm_, torque_limit_nm_);
    vel_pid_.saturated = (std::abs(torque) >= torque_limit_nm_);
    return torque;
  }

  // ── run_outer_loop — position PID → v_cmd_, rate-divided ──────────────────────────────────────
  void run_outer_loop(double pos_ref, double vel_ff, double dt)
  {
    outer_dt_accum_ += dt;
    if (++outer_loop_counter_ >= static_cast<int>(std::llround(outer_loop_divider_))) {
      outer_loop_counter_ = 0;
      double pid_out = pos_pid_.compute(pos_ref, last_measured_pos_, outer_dt_accum_);
      v_cmd_ = std::clamp(pid_out + vel_ff, -pos_output_limit_, pos_output_limit_);
      pos_pid_.saturated = (std::abs(v_cmd_) >= pos_output_limit_);
      outer_dt_accum_ = 0.0;
    }
  }

  // ── publish_diagnostics — shared Float64 publishing after mode dispatch ────────────────────────
  void publish_diagnostics(double vel_ref, double vel_error, double pos_ref, double pos_error)
  {
    pub_f64(desired_vel_pub_,       vel_ref);
    pub_f64(measured_vel_pub_,      last_measured_vel_);
    pub_f64(velocity_error_pub_,    vel_error);
    pub_f64(measured_position_pub_, last_measured_pos_);
    pub_f64(position_error_pub_,    pos_error);
    pub_f64(position_command_pub_,  pos_ref);
    pub_f64(velocity_command_pub_,  v_cmd_);
  }

  // ══════════════════════════════════════════════════════════════════════════════════════════════
  //  Control loop
  // ══════════════════════════════════════════════════════════════════════════════════════════════

  void control_loop()
  {
    if (joint_index_ < 0) { publish_torque(0.0); return; }

    const auto now = this->now();
    const double actual_dt = (now - last_loop_time_).seconds();
    last_loop_time_ = now;
    const double dt = (actual_dt > 0.0) ? actual_dt : dt_;
    const double t  = (now - start_time_).seconds();

    const auto traj = compute_trajectory(t);

    double torque    = 0.0;
    double vel_error = 0.0;
    double pos_error = traj.pos - last_measured_pos_;

    if (control_mode_ == "cascade") {
      run_outer_loop(traj.pos, traj.vel, dt);
      torque    = run_inner_loop(v_cmd_, traj.accel, dt);
      vel_error = v_cmd_ - last_measured_vel_;
    } else if (control_mode_ == "position_only") {
      torque = pos_pid_.compute(traj.pos, last_measured_pos_, dt);
      torque = std::clamp(torque, -torque_limit_nm_, torque_limit_nm_);
      pos_pid_.saturated = (std::abs(torque) >= torque_limit_nm_);
      v_cmd_ = 0.0;
    } else {  // velocity_only
      v_cmd_    = traj.vel;
      torque    = run_inner_loop(traj.vel, traj.accel, dt);
      vel_error = traj.vel - last_measured_vel_;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
      "[%s] pos_err=%.3f v_cmd=%.3f vel_err=%.3f torque=%.3f sat=%d",
      control_mode_.c_str(), pos_error, v_cmd_, vel_error, torque,
      static_cast<int>(vel_pid_.saturated));

    publish_torque(torque);
    publish_diagnostics(traj.vel, vel_error, traj.pos, pos_error);
  }

  void publish_torque(double torque)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {torque};
    torque_pub_->publish(msg);
  }

  // ── ROS interfaces ────────────────────────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measured_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;

  // ── Parameters ────────────────────────────────────────────────────────────────────────────────
  std::string joint_state_topic_;
  std::string command_topic_;
  std::string joint_name_;
  std::string control_mode_;
  // Inner loop (velocity PID)
  double amplitude_rad_s_;
  double omega_rad_s_;
  double kp_;
  double ki_;
  double kd_;
  double kff_;
  double kaff_;
  double torque_limit_nm_;
  double integral_limit_;
  double deadband_rad_s_;
  // Outer loop (position PID)
  double kp_pos_;
  double ki_pos_;
  double kd_pos_;
  double pos_integral_limit_;
  double pos_output_limit_;
  double outer_loop_divider_;
  // Misc
  double rate_hz_;
  double dt_;
  double filter_alpha_;
  bool   invert_output_;
  double position_setpoint_;

  // ── State ─────────────────────────────────────────────────────────────────────────────────────
  double last_measured_vel_;
  double filtered_vel_;
  bool   filter_initialized_;
  int    joint_index_;
  double last_measured_pos_;
  rclcpp::Time start_time_;
  rclcpp::Time last_loop_time_;
  // Cascade / outer-loop state
  int    outer_loop_counter_;
  double outer_dt_accum_;
  double v_cmd_;   // current velocity command (output of outer loop; held between outer ticks)

  // ── PID controllers ───────────────────────────────────────────────────────────────────────────
  PidController vel_pid_;   // inner loop: velocity → torque
  PidController pos_pid_;   // outer loop: position → velocity command

  // ── Parameter table ───────────────────────────────────────────────────────────────────────────
  std::vector<ParamDef> param_defs_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPidNode>());
  rclcpp::shutdown();
  return 0;
}
