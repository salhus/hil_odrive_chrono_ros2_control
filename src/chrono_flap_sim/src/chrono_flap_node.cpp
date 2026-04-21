// chrono_flap_node.cpp
//
// ChronoFlapNode — Project Chrono inverted-pendulum flap simulation.
//
// Two operating modes:
//
//   sil_mode = true   Software-in-the-Loop: Chrono publishes /joint_states so the
//                     velocity_pid_node closes the loop through the sim. No hardware.
//
//   sil_mode = false  Parallel/shadow: Chrono shadows the real hardware.
//                     With use_shadow_pid=true (default), an internal closed-loop shadow PID
//                     drives the sim using sim_position / sim_velocity as feedback, making it
//                     inherently stable.  With use_shadow_pid=false, the torque command from
//                     the real controller is fed open-loop into Chrono.
//
// Torque law each sub-step:
//   τ_total = τ_cmd − (B_joint + B_bearing)·ω − C_coulomb·sign(ω) − K·θ
//
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// Project Chrono headers
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/functions/ChFunctionSetpoint.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/ChConfig.h"
// Visualization (optional - conditional compilation).
// Use CHRONO_FLAP_USE_VSG / CHRONO_FLAP_USE_IRRLICHT (set by our CMakeLists.txt)
// instead of CHRONO_VSG / CHRONO_IRRLICHT so the guards are decoupled from
// whatever Chrono's cmake config injects via its imported targets.
#if defined(CHRONO_FLAP_USE_VSG)
#  include "chrono_vsg/ChVisualSystemVSG.h"
   using namespace chrono::vsg3d;
#elif defined(CHRONO_FLAP_USE_IRRLICHT)
#  include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
   using namespace chrono::irrlicht;
#endif
// Shadow PID controller (reuses PidController from odrive_velocity_pid)
#include "chrono_flap_sim/shadow_pid_controller.hpp"

using namespace chrono;

static constexpr double kFlapVisDepth    = 0.0025;  // 0.25 cm acrylic thickness
static constexpr double kPivotMarkerSize = 0.04;

class ChronoFlapNode : public rclcpp::Node
{
public:
  ChronoFlapNode()
  : Node("chrono_flap_node"),
    latest_torque_(0.0),
    sim_position_(0.0),
    sim_velocity_(0.0),
    sim_acceleration_(0.0)
  {
    // ── Mode & identity ──────────────────────────────────────────────────────────────────────
    this->declare_parameter<bool>("sil_mode", false);
    this->declare_parameter<std::string>("joint_name", "motor_joint");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("effort_topic", "/motor_effort_controller/commands");
    this->declare_parameter<bool>("enable_visualization", false);

    // ── Timing ───────────────────────────────────────────────────────────────────────────────
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<double>("solver_rate_hz", 1000.0);

    // ── Physical flap properties ─────────────────────────────────────────────────────────────
    this->declare_parameter<double>("flap_length_m", 0.30);
    this->declare_parameter<double>("flap_width_m", 0.30);
    this->declare_parameter<double>("flap_mass_kg", 0.21);

    // ── Joint dynamics ───────────────────────────────────────────────────────────────────────
    this->declare_parameter<double>("joint_damping", 0.0);
    this->declare_parameter<double>("joint_stiffness", 0.712441);
    this->declare_parameter<double>("bearing_friction", 0.4);
    this->declare_parameter<double>("coulomb_friction", 0.0);

    // ── Shadow PID toggle ────────────────────────────────────────────────────────────────────
    this->declare_parameter<bool>("use_shadow_pid", true);

    // ── Shadow PID control mode ──────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("shadow_control_mode", "position_only");

    // ── Shadow PID trajectory sync ───────────────────────────────────────────────────────────
    // When true (default in parallel mode), shadow PID subscribes to velocity_pid_node's
    // published position_command / velocity_command topics instead of generating its own
    // trajectory. Forced false in SIL mode to avoid circular dependency.
    this->declare_parameter<bool>("shadow_sync_trajectory", true);

    // ── Shadow PID trajectory (used only when shadow_sync_trajectory=false) ─────────────────
    this->declare_parameter<double>("shadow_position_setpoint", 0.0);
    this->declare_parameter<double>("shadow_amplitude_rad_s",   0.0);
    this->declare_parameter<double>("shadow_omega_rad_s",       0.0);

    // ── Shadow PID inner loop (velocity) ─────────────────────────────────────────────────────
    this->declare_parameter<double>("shadow_kp",              0.35);
    this->declare_parameter<double>("shadow_ki",              0.01);
    this->declare_parameter<double>("shadow_kd",              0.00);
    this->declare_parameter<double>("shadow_kff",             0.40);
    this->declare_parameter<double>("shadow_kaff",            0.20);
    this->declare_parameter<double>("shadow_torque_limit_nm", 0.40);
    this->declare_parameter<double>("shadow_integral_limit",  0.00);
    this->declare_parameter<double>("shadow_deadband_rad_s",  0.00);

    // ── Shadow PID outer loop (position) ─────────────────────────────────────────────────────
    this->declare_parameter<double>("shadow_kp_pos",           2.00);
    this->declare_parameter<double>("shadow_ki_pos",           0.01);
    this->declare_parameter<double>("shadow_kd_pos",          0.025);
    this->declare_parameter<double>("shadow_pos_integral_limit", 1.00);
    this->declare_parameter<double>("shadow_pos_output_limit",   2.00);

    // ── Shadow PID misc ──────────────────────────────────────────────────────────────────────
    this->declare_parameter<double>("shadow_outer_loop_divider", 1.0);
    this->declare_parameter<double>("shadow_filter_alpha",       0.90);

    // ── Read parameters ───────────────────────────────────────────────────────────────────────
    sil_mode_          = this->get_parameter("sil_mode").as_bool();
    joint_name_        = this->get_parameter("joint_name").as_string();
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    effort_topic_      = this->get_parameter("effort_topic").as_string();
    enable_vis_        = this->get_parameter("enable_visualization").as_bool();
    rate_hz_           = this->get_parameter("rate_hz").as_double();
    solver_rate_hz_    = this->get_parameter("solver_rate_hz").as_double();
    flap_length_       = this->get_parameter("flap_length_m").as_double();
    flap_width_        = this->get_parameter("flap_width_m").as_double();
    flap_mass_         = this->get_parameter("flap_mass_kg").as_double();
    joint_damping_     = this->get_parameter("joint_damping").as_double();
    joint_stiffness_   = this->get_parameter("joint_stiffness").as_double();
    bearing_friction_  = this->get_parameter("bearing_friction").as_double();
    coulomb_friction_  = this->get_parameter("coulomb_friction").as_double();
    use_shadow_pid_    = this->get_parameter("use_shadow_pid").as_bool();

    shadow_sync_trajectory_ = this->get_parameter("shadow_sync_trajectory").as_bool();
    // No longer force-disabled in SIL mode — shadow PID drives the plant
    // in both modes, so syncing trajectory from velocity_pid_node is safe
    // (velocity_pid_node's trajectory generator is not feedback-dependent).

    read_shadow_params();

    // Timing derived quantities
    publish_dt_ = (rate_hz_ > 0.0) ? (1.0 / rate_hz_) : 0.01;
    solver_dt_  = (solver_rate_hz_ > 0.0) ? (1.0 / solver_rate_hz_) : 0.001;
    substeps_   = std::max(1, static_cast<int>(std::round(publish_dt_ / solver_dt_)));

    build_chrono_system();

    // ── ROS interfaces ────────────────────────────────────────────────────────────────────────
    effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      effort_topic_,
      rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          latest_torque_ = msg->data[0];
        }
      });

    pos_pub_          = this->create_publisher<std_msgs::msg::Float64>("~/sim_position",     10);
    vel_pub_          = this->create_publisher<std_msgs::msg::Float64>("~/sim_velocity",     10);
    accel_pub_        = this->create_publisher<std_msgs::msg::Float64>("~/sim_acceleration", 10);
    shadow_torque_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/shadow_torque",   10);

    if (sil_mode_) {
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10);
      RCLCPP_INFO(this->get_logger(), "SIL mode: publishing %s (joint='%s')",
        joint_state_topic_.c_str(), joint_name_.c_str());
    } else {
      // Parallel mode: publish sim joint states on a separate topic for the sim RSP / RViz overlay
      sim_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/sim_joint_states", 10);
    }

    // Trajectory sync — works in both modes now
    if (shadow_sync_trajectory_) {
      pos_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/velocity_pid_node/position_command",
        rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          shadow_ext_pos_ref_ = msg->data;
          shadow_pos_ref_received_ = true;
        });
      vel_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/velocity_pid_node/velocity_command",
        rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          shadow_ext_vel_ref_ = msg->data;
          shadow_vel_ref_received_ = true;
        });
      RCLCPP_INFO(this->get_logger(),
        "Shadow PID will sync trajectory from /velocity_pid_node/{position,velocity}_command");
    }

    // ── Parameter callbacks ───────────────────────────────────────────────────────────────────
    on_set_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_validate_parameters(params);
      });
    post_set_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        on_apply_parameters(params);
      });

    start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "ChronoFlapNode started [sil_mode=%s, use_shadow_pid=%s, shadow_sync_trajectory=%s]: "
      "publish=%.0f Hz, solver=%.0f Hz (%d substeps), "
      "flap=%.3fx%.3f m / %.4f kg, damping=%.4f, stiffness=%.4f, bearing=%.4f, coulomb=%.4f",
      sil_mode_ ? "true" : "false", use_shadow_pid_ ? "true" : "false",
      shadow_sync_trajectory_ ? "true" : "false",
      rate_hz_, solver_rate_hz_, substeps_,
      flap_length_, flap_width_, flap_mass_,
      joint_damping_, joint_stiffness_, bearing_friction_, coulomb_friction_);
  }

  void run()
  {
    if (enable_vis_) {
      init_visualization();
    } else {
      RCLCPP_INFO(this->get_logger(), "Visualization disabled by parameter.");
    }

    using clock = std::chrono::steady_clock;
    const auto step_duration = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(publish_dt_));

    while (rclcpp::ok()) {
      auto t_start = clock::now();
      rclcpp::spin_some(this->shared_from_this());

      if (vis_active_) {
#if defined(CHRONO_FLAP_USE_VSG) || defined(CHRONO_FLAP_USE_IRRLICHT)
        if (vis_) {
          if (!vis_->Run()) { break; }
          vis_->BeginScene();
          vis_->Render();
          vis_->EndScene();
        }
#endif
      }

      // Determine torque command for this tick
      double torque_cmd = 0.0;
      double shadow_torque = 0.0;
      if (use_shadow_pid_) {
        if (shadow_sync_trajectory_) {
          if (shadow_pos_ref_received_ && shadow_vel_ref_received_) {
            // Use references synced from velocity_pid_node topics
            shadow_torque = shadow_pid_.compute(
              sim_position_, sim_velocity_, shadow_ext_pos_ref_, shadow_ext_vel_ref_, publish_dt_);
          } else {
            // External refs not yet received — hold at zero until velocity_pid_node is ready
            shadow_torque = shadow_pid_.compute(
              sim_position_, sim_velocity_, 0.0, 0.0, publish_dt_);
          }
        } else {
          const double t = (this->now() - start_time_).seconds();
          shadow_torque = shadow_pid_.compute(sim_position_, sim_velocity_, t, publish_dt_);
        }
        torque_cmd    = shadow_torque;
      } else {
        torque_cmd = latest_torque_;
      }

      // Record velocity before sub-stepping for acceleration estimate
      const double vel_before = sim_velocity_;

      // Sub-step loop: τ_total = τ_cmd − (B_joint + B_bearing)·ω − C_coulomb·sign(ω) − K·θ
      for (int i = 0; i < substeps_; ++i) {
        const double angle  = motor_link_->GetMotorAngle();
        const double omega  = motor_link_->GetMotorAngleDt();
        const double total_damping = joint_damping_ + bearing_friction_;
        const double coulomb = coulomb_friction_ * (omega > 0.0 ? 1.0 : (omega < 0.0 ? -1.0 : 0.0));
        const double total_torque = torque_cmd
                                   - total_damping  * omega
                                   - coulomb
                                   - joint_stiffness_ * angle;
        torque_fn_->SetSetpoint(total_torque, sys_->GetChTime());
        sys_->DoStepDynamics(solver_dt_);
      }

      sim_position_     = motor_link_->GetMotorAngle();
      sim_velocity_     = motor_link_->GetMotorAngleDt();
      sim_acceleration_ = (sim_velocity_ - vel_before) / publish_dt_;

      publish_kinematics(shadow_torque);

      if (sil_mode_) {
        publish_joint_state();
      } else {
        publish_sim_joint_state();
      }

      auto elapsed = clock::now() - t_start;
      if (elapsed < step_duration) {
        std::this_thread::sleep_for(step_duration - elapsed);
      }
    }
  }

private:
  // ── Build Chrono rigid-body system ──────────────────────────────────────────────────────────
  void build_chrono_system()
  {
    sys_ = std::make_unique<ChSystemNSC>();
    sys_->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ground_ = std::make_shared<ChBody>();
    ground_->SetFixed(true);
    ground_->AddVisualShape(std::make_shared<ChVisualShapeBox>(
      kPivotMarkerSize, kPivotMarkerSize, kPivotMarkerSize));
    sys_->AddBody(ground_);

    flap_ = std::make_shared<ChBody>();
    update_flap_inertia();
    flap_vis_shape_ = std::make_shared<ChVisualShapeBox>(
      flap_width_, kFlapVisDepth, flap_length_);
    flap_->AddVisualShape(flap_vis_shape_);
    sys_->AddBody(flap_);

    motor_link_ = std::make_shared<ChLinkMotorRotationTorque>();
    torque_fn_  = std::make_shared<ChFunctionSetpoint>();
    motor_link_->SetTorqueFunction(torque_fn_);
    // Revolute axis = Y so the flap swings in the XZ plane under gravity.
    ChFrame<> pivot_frame(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2));
    motor_link_->Initialize(flap_, ground_, pivot_frame);
    sys_->AddLink(motor_link_);
  }

  // ── Visualization init ───────────────────────────────────────────────────────────────────────
  void init_visualization()
  {
#if defined(CHRONO_FLAP_USE_VSG)
    try {
      auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
      vis->AttachSystem(sys_.get());
      vis->SetWindowTitle("Chrono Flap Simulation (Inverted Pendulum)");
      vis->SetWindowSize(800, 600);
      vis->AddCamera(ChVector3d(0.0, -1.5, 0.0), ChVector3d(0.0, 0.0, 0.0));
      vis->Initialize();
      vis_ = vis;
      vis_active_ = true;
      RCLCPP_INFO(this->get_logger(), "VSG visualization initialized.");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(),
        "VSG initialization failed (%s) - falling back to headless.", e.what());
      vis_active_ = false;
    } catch (...) {
      RCLCPP_WARN(this->get_logger(),
        "VSG initialization failed (unknown error) - falling back to headless.");
      vis_active_ = false;
    }
#elif defined(CHRONO_FLAP_USE_IRRLICHT)
    try {
      auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
      vis->SetWindowSize(800, 600);
      vis->SetWindowTitle("Chrono Flap Simulation (Inverted Pendulum)");
      vis->Initialize();
      vis->AddSkyBox();
      vis->AddCamera(ChVector3d(0.0, -1.5, 0.0), ChVector3d(0.0, 0.0, 0.0));
      vis->AddTypicalLights();
      vis->AttachSystem(sys_.get());
      vis_ = vis;
      vis_active_ = true;
      RCLCPP_INFO(this->get_logger(), "Irrlicht visualization initialized.");
    } catch (...) {
      RCLCPP_WARN(this->get_logger(),
        "Irrlicht initialization failed - falling back to headless.");
      vis_active_ = false;
    }
#else
    RCLCPP_INFO(this->get_logger(), "Running headless (no visualization compiled).");
    vis_active_ = false;
#endif
  }

  // ── Update flap inertia tensor ───────────────────────────────────────────────────────────────
  void update_flap_inertia()
  {
    flap_->SetMass(flap_mass_);
    const double L = flap_length_;
    const double W = flap_width_;
    const double m = flap_mass_;
    const double I_pivot = (1.0 / 3.0) * m * L * L;
    const double I_width = (1.0 / 12.0) * m * W * W;
    const double I_thin  = 1e-6 * I_pivot;
    flap_->SetInertiaXX(ChVector3d(I_pivot, I_width, I_thin));
    flap_->SetPos(ChVector3d(0.0, 0.0, L / 2.0));
    flap_->SetPosDt(ChVector3d(0.0, 0.0, 0.0));
    flap_->SetAngVelParent(ChVector3d(0.0, 0.0, 0.0));
  }

  // ── Read shadow PID params from ROS parameter store → ShadowPidController ──────────────────
  void read_shadow_params()
  {
    const std::string cm = this->get_parameter("shadow_control_mode").as_string();
    shadow_pid_.control_mode = is_valid_control_mode(cm) ? cm : "position_only";

    shadow_pid_.position_setpoint  = this->get_parameter("shadow_position_setpoint").as_double();
    shadow_pid_.amplitude_rad_s    = this->get_parameter("shadow_amplitude_rad_s").as_double();
    shadow_pid_.omega_rad_s        = this->get_parameter("shadow_omega_rad_s").as_double();
    shadow_pid_.kp                 = this->get_parameter("shadow_kp").as_double();
    shadow_pid_.ki                 = this->get_parameter("shadow_ki").as_double();
    shadow_pid_.kd                 = this->get_parameter("shadow_kd").as_double();
    shadow_pid_.kff                = this->get_parameter("shadow_kff").as_double();
    shadow_pid_.kaff               = this->get_parameter("shadow_kaff").as_double();
    shadow_pid_.torque_limit_nm    = this->get_parameter("shadow_torque_limit_nm").as_double();
    shadow_pid_.integral_limit     = this->get_parameter("shadow_integral_limit").as_double();
    shadow_pid_.deadband_rad_s     = this->get_parameter("shadow_deadband_rad_s").as_double();
    shadow_pid_.kp_pos             = this->get_parameter("shadow_kp_pos").as_double();
    shadow_pid_.ki_pos             = this->get_parameter("shadow_ki_pos").as_double();
    shadow_pid_.kd_pos             = this->get_parameter("shadow_kd_pos").as_double();
    shadow_pid_.pos_integral_limit = this->get_parameter("shadow_pos_integral_limit").as_double();
    shadow_pid_.pos_output_limit   = this->get_parameter("shadow_pos_output_limit").as_double();
    shadow_pid_.outer_loop_divider = this->get_parameter("shadow_outer_loop_divider").as_double();
    shadow_pid_.filter_alpha       = this->get_parameter("shadow_filter_alpha").as_double();
    shadow_pid_.reset();
  }

  // ── Parameter validation ─────────────────────────────────────────────────────────────────────
  static bool is_valid_control_mode(const std::string & mode)
  {
    return mode == "velocity_only" || mode == "cascade" || mode == "position_only";
  }

  static inline const std::set<std::string> kImmutableParams = {
    "rate_hz", "solver_rate_hz", "effort_topic", "sil_mode", "joint_name", "joint_state_topic"};

  rcl_interfaces::msg::SetParametersResult on_validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (kImmutableParams.count(param.get_name())) {
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() + "' cannot be changed at runtime.";
        return result;
      }

      // shadow_control_mode validation
      if (param.get_name() == "shadow_control_mode") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            !is_valid_control_mode(param.as_string())) {
          result.successful = false;
          result.reason = "shadow_control_mode must be 'cascade', 'velocity_only', or 'position_only'.";
          return result;
        }
      }

      // Positive-valued flap geometry
      if (param.get_name() == "flap_length_m" || param.get_name() == "flap_mass_kg"
          || param.get_name() == "flap_width_m") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be a positive double.";
          return result;
        }
      }

      // Non-negative joint dynamics
      if (param.get_name() == "joint_damping" || param.get_name() == "joint_stiffness"
          || param.get_name() == "bearing_friction" || param.get_name() == "coulomb_friction") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || param.as_double() < 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be non-negative.";
          return result;
        }
      }

      // Shadow PID positive limits
      static const std::set<std::string> kShadowPositive = {
        "shadow_torque_limit_nm", "shadow_pos_integral_limit",
        "shadow_pos_output_limit", "shadow_outer_loop_divider"};
      if (kShadowPositive.count(param.get_name())) {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be positive.";
          return result;
        }
      }

      // filter_alpha in [0, 1)
      if (param.get_name() == "shadow_filter_alpha") {
        const double v = param.as_double();
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || v < 0.0 || v >= 1.0) {
          result.successful = false;
          result.reason = "shadow_filter_alpha must be in [0.0, 1.0).";
          return result;
        }
      }
    }
    return result;
  }

  // ── Parameter application ────────────────────────────────────────────────────────────────────
  void on_apply_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool body_needs_update  = false;
    bool shadow_pid_changed = false;

    for (const auto & param : parameters) {
      const auto & n = param.get_name();

      // Physical parameters
      if (n == "flap_length_m")     { flap_length_ = param.as_double();    body_needs_update = true; }
      else if (n == "flap_width_m") { flap_width_  = param.as_double();    body_needs_update = true; }
      else if (n == "flap_mass_kg") { flap_mass_   = param.as_double();    body_needs_update = true; }
      else if (n == "joint_damping")    { joint_damping_    = param.as_double(); }
      else if (n == "joint_stiffness")  { joint_stiffness_  = param.as_double(); }
      else if (n == "bearing_friction") { bearing_friction_ = param.as_double(); }
      else if (n == "coulomb_friction") { coulomb_friction_ = param.as_double(); }
      else if (n == "use_shadow_pid")   { use_shadow_pid_ = param.as_bool(); shadow_pid_changed = true; }

      // Shadow PID parameters — any change triggers re-read and reset
      else if (n.rfind("shadow_", 0) == 0) { shadow_pid_changed = true; }
    }

    if (body_needs_update) {
      update_flap_inertia();
      if (flap_->GetVisualModel()) { flap_->GetVisualModel()->Clear(); }
      flap_vis_shape_ = std::make_shared<ChVisualShapeBox>(
        flap_width_, kFlapVisDepth, flap_length_);
      flap_->AddVisualShape(flap_vis_shape_);
    }

    if (shadow_pid_changed) {
      read_shadow_params();
    }
  }

  // ── Publishing ───────────────────────────────────────────────────────────────────────────────
  void publish_kinematics(double shadow_torque)
  {
    auto pub_f64 = [](rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double val) {
      std_msgs::msg::Float64 m;
      m.data = val;
      pub->publish(m);
    };
    pub_f64(pos_pub_,           sim_position_);
    pub_f64(vel_pub_,           sim_velocity_);
    pub_f64(accel_pub_,         sim_acceleration_);
    pub_f64(shadow_torque_pub_, shadow_torque);
  }

  void publish_joint_state()
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.push_back(joint_name_);
    js.position.push_back(sim_position_);
    js.velocity.push_back(sim_velocity_);
    js.effort.push_back(latest_torque_);
    // pto_link/pto_joint commented out from URDF — no longer needed here
    // js.name.push_back("pto_joint");
    // js.position.push_back(0.0);
    // js.velocity.push_back(0.0);
    // js.effort.push_back(0.0);
    joint_state_pub_->publish(js);
  }

  // Publish sim joint states on /sim_joint_states for the second RSP (RViz overlay) in parallel mode
  void publish_sim_joint_state()
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.push_back(joint_name_);
    js.position.push_back(sim_position_);
    js.velocity.push_back(sim_velocity_);
    js.effort.push_back(0.0);
    // pto_link/pto_joint commented out from URDF — no longer needed here
    // js.name.push_back("pto_joint");
    // js.position.push_back(0.0);
    // js.velocity.push_back(0.0);
    // js.effort.push_back(0.0);
    sim_joint_state_pub_->publish(js);
  }

  // ── Member variables ─────────────────────────────────────────────────────────────────────────

  // Mode & identity
  bool        sil_mode_{false};
  std::string joint_name_{"motor_joint"};
  std::string joint_state_topic_{"/joint_states"};
  std::string effort_topic_{"/motor_effort_controller/commands"};
  bool        enable_vis_{false};
  bool        vis_active_{false};

  // Timing
  double rate_hz_{100.0};
  double solver_rate_hz_{1000.0};
  double publish_dt_{0.01};
  double solver_dt_{0.001};
  int    substeps_{10};

  // Physical properties
  double flap_length_{0.30};
  double flap_width_{0.30};
  double flap_mass_{0.21};
  double joint_damping_{0.0};
  double joint_stiffness_{0.712441};
  double bearing_friction_{0.4};
  double coulomb_friction_{0.0};

  // Shadow PID
  bool              use_shadow_pid_{true};
  bool              shadow_sync_trajectory_{true};
  double            shadow_ext_pos_ref_{0.0};
  double            shadow_ext_vel_ref_{0.0};
  bool              shadow_pos_ref_received_{false};
  bool              shadow_vel_ref_received_{false};
  ShadowPidController shadow_pid_;

  // Runtime state
  double latest_torque_;
  double sim_position_;
  double sim_velocity_;
  double sim_acceleration_;
  rclcpp::Time start_time_;

  // Chrono objects
  std::unique_ptr<ChSystemNSC>               sys_;
  std::shared_ptr<ChBody>                    ground_;
  std::shared_ptr<ChBody>                    flap_;
  std::shared_ptr<ChLinkMotorRotationTorque> motor_link_;
  std::shared_ptr<ChFunctionSetpoint>        torque_fn_;
  std::shared_ptr<ChVisualShapeBox>          flap_vis_shape_;
#if defined(CHRONO_FLAP_USE_VSG) || defined(CHRONO_FLAP_USE_IRRLICHT)
  std::shared_ptr<ChVisualSystem>            vis_;
#endif

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           pos_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           vel_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              accel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              shadow_torque_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        sim_joint_state_pub_;

  // Parameter callback handles
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr   on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChronoFlapNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
