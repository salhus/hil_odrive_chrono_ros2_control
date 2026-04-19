// chrono_flap_node.cpp

// ChronoFlapNode — Project Chrono inverted-pendulum flap simulation with two modes:
//
//   sim_mode = "sil"      Software-in-the-Loop: Chrono publishes /joint_states so the
//                         velocity_pid_node closes the loop through the sim. No hardware.
//
//   sim_mode = "parallel" Parallel/shadow: Chrono reads the same torque the real ODrive
//                         receives but does NOT publish /joint_states. The real hardware
//                         owns the control loop. Compare sim vs real in PlotJuggler.
//
// Physical setup:
//   - 30 cm × 30 cm × 0.25 cm acrylic flap (~0.27 kg)
//   - Pivot at the BOTTOM edge (inverted pendulum — flap stands upright)
//   - Powered ODrive on one end of the pivot axis
//   - Unpowered ODrive on the other end (passive bearing with cogging/friction)
//   - θ = 0 is upright; gravity destabilises the flap
//
// Torque law each sub-step:
//   τ_total = τ_external − (B_joint + B_bearing)·ω − K·θ
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
// Visualization (optional - conditional compilation)
#if defined(CHRONO_VSG)
#  include "chrono_vsg/ChVisualSystemVSG.h"
   using namespace chrono::vsg3d;
#elif defined(CHRONO_IRRLICHT)
#  include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
   using namespace chrono::irrlicht;
#endif
using namespace chrono;
// Visual shape constants
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
    // --- Declare parameters ---
    // Mode
    this->declare_parameter<bool>("sil_mode", false);
    this->declare_parameter<std::string>("joint_name", "motor_joint");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    // Timing
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<double>("solver_rate_hz", 1000.0);
    // Physical flap properties (30 cm × 30 cm × 0.25 cm acrylic)
    this->declare_parameter<double>("flap_length_m", 0.30);
    this->declare_parameter<double>("flap_width_m", 0.30);
    this->declare_parameter<double>("flap_mass_kg", 0.5);
    // Joint dynamics
    this->declare_parameter<double>("joint_damping", 0.0);
    this->declare_parameter<double>("joint_stiffness", 0.712441);
    this->declare_parameter<double>("bearing_friction", 0.005);
    // ROS interface
    this->declare_parameter<std::string>("effort_topic", "/motor_effort_controller/commands");
    this->declare_parameter<bool>("enable_visualization", false);

    // --- Read parameters ---
    sil_mode_          = this->get_parameter("sil_mode").as_bool();
    joint_name_        = this->get_parameter("joint_name").as_string();
    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    rate_hz_           = this->get_parameter("rate_hz").as_double();
    solver_rate_hz_    = this->get_parameter("solver_rate_hz").as_double();
    flap_length_       = this->get_parameter("flap_length_m").as_double();
    flap_width_        = this->get_parameter("flap_width_m").as_double();
    flap_mass_         = this->get_parameter("flap_mass_kg").as_double();
    joint_damping_     = this->get_parameter("joint_damping").as_double();
    joint_stiffness_   = this->get_parameter("joint_stiffness").as_double();
    bearing_friction_  = this->get_parameter("bearing_friction").as_double();
    effort_topic_      = this->get_parameter("effort_topic").as_string();
    enable_vis_        = this->get_parameter("enable_visualization").as_bool();

    // Publish interval (wall-clock pacing)
    publish_dt_ = (rate_hz_ > 0.0) ? (1.0 / rate_hz_) : 0.01;
    // Solver timestep (internal sub-stepping)
    solver_dt_ = (solver_rate_hz_ > 0.0) ? (1.0 / solver_rate_hz_) : 0.001;
    // Number of solver sub-steps per publish tick (at least 1)
    substeps_ = std::max(1, static_cast<int>(std::round(publish_dt_ / solver_dt_)));

    build_chrono_system();

    // --- ROS interfaces ---
    effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      effort_topic_,
      rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (!msg->data.empty()) {
          latest_torque_ = msg->data[0];
        }
      });

    pos_pub_   = this->create_publisher<std_msgs::msg::Float64>("~/sim_position",     10);
    vel_pub_   = this->create_publisher<std_msgs::msg::Float64>("~/sim_velocity",     10);
    accel_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/sim_acceleration", 10);

    // In SIL mode, publish JointState so the PID node closes the loop through Chrono
    if (sil_mode_) {
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10);
      RCLCPP_INFO(this->get_logger(), "SIL mode: publishing %s (joint='%s')",
        joint_state_topic_.c_str(), joint_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Parallel mode: NOT publishing /joint_states");
    }

    // --- Parameter callbacks ---
    on_set_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_validate_parameters(params);
      });
    post_set_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        on_apply_parameters(params);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "ChronoFlapNode started [sil_mode=%s]: publish=%.0f Hz, solver=%.0f Hz (%d substeps), "
      "flap=%.3fx%.3f m / %.4f kg, damping=%.4f, stiffness=%.4f, bearing=%.4f",
      sil_mode_ ? "true" : "false", rate_hz_, solver_rate_hz_, substeps_,
      flap_length_, flap_width_, flap_mass_,
      joint_damping_, joint_stiffness_, bearing_friction_);
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
#if defined(CHRONO_VSG) || defined(CHRONO_IRRLICHT)
        if (vis_) {
          if (!vis_->Run())
            break;
          vis_->BeginScene();
          vis_->Render();
          vis_->EndScene();
        }
#endif
      }
      // Record velocity before sub-stepping for acceleration estimate
      const double vel_before = sim_velocity_;
      // Run solver sub-steps
      // τ_total = τ_external - (B_joint + B_bearing)·ω - K·θ
      for (int i = 0; i < substeps_; ++i) {
        const double angle = motor_link_->GetMotorAngle();
        const double omega = motor_link_->GetMotorAngleDt();
        const double total_damping = joint_damping_ + bearing_friction_;
        const double total_torque = latest_torque_
                                  - total_damping    * omega
                                  - joint_stiffness_ * angle;
        torque_fn_->SetSetpoint(total_torque, sys_->GetChTime());
        sys_->DoStepDynamics(solver_dt_);
      }
      sim_position_ = motor_link_->GetMotorAngle();
      sim_velocity_ = motor_link_->GetMotorAngleDt();
      sim_acceleration_ = (sim_velocity_ - vel_before) / publish_dt_;
      publish_kinematics();
      // In SIL mode, publish JointState so the PID reads from us
      if (sil_mode_) {
        publish_joint_state();
      }
      auto elapsed = clock::now() - t_start;
      if (elapsed < step_duration) {
        std::this_thread::sleep_for(step_duration - elapsed);
      }
    }
  }
private:
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
    // QuatFromAngleX(CH_PI_2) rotates the default Z motor axis to Y.
    ChFrame<> pivot_frame(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2));
    motor_link_->Initialize(flap_, ground_, pivot_frame);
    sys_->AddLink(motor_link_);
  }
  void init_visualization()
  {
#if defined(CHRONO_VSG)
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
#elif defined(CHRONO_IRRLICHT)
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
  void update_flap_inertia()
  {
    flap_->SetMass(flap_mass_);
    const double L = flap_length_;
    const double W = flap_width_;
    const double m = flap_mass_;
    // Moment of inertia of a thin rectangular plate about the bottom edge (pivot):
    // I_pivot = (1/3)*m*L^2  (rotation about the pivot axis = Y)
    const double I_pivot = (1.0 / 3.0) * m * L * L;
    const double I_width = (1.0 / 12.0) * m * W * W;
    const double I_thin  = 1e-6 * I_pivot;
    // Inertia tensor at CoM (Chrono uses principal axes at CoM)
    // X = width direction, Y = pivot axis, Z = length direction (up)
    flap_->SetInertiaXX(ChVector3d(I_pivot, I_width, I_thin));
    // Flap extends UPWARD (+Z) from pivot; CoM at L/2 above pivot
    // theta = 0 is upright (inverted pendulum)
    flap_->SetPos(ChVector3d(0.0, 0.0, L / 2.0));
    flap_->SetPosDt(ChVector3d(0.0, 0.0, 0.0));
    flap_->SetAngVelParent(ChVector3d(0.0, 0.0, 0.0));
  }
  // --- Parameter validation & application ---
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
      if (param.get_name() == "flap_length_m" || param.get_name() == "flap_mass_kg"
          || param.get_name() == "flap_width_m") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || param.as_double() <= 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be a positive double.";
          return result;
        }
      }
      if (param.get_name() == "joint_damping" || param.get_name() == "joint_stiffness"
          || param.get_name() == "bearing_friction") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || param.as_double() < 0.0) {
          result.successful = false;
          result.reason = param.get_name() + " must be non-negative.";
          return result;
        }
      }
    }
    return result;
  }
  void on_apply_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool body_needs_update = false;
    for (const auto & param : parameters) {
      if (param.get_name() == "flap_length_m")     { flap_length_ = param.as_double(); body_needs_update = true; }
      else if (param.get_name() == "flap_width_m") { flap_width_ = param.as_double(); body_needs_update = true; }
      else if (param.get_name() == "flap_mass_kg") { flap_mass_ = param.as_double(); body_needs_update = true; }
      else if (param.get_name() == "joint_damping")    { joint_damping_ = param.as_double(); }
      else if (param.get_name() == "joint_stiffness")  { joint_stiffness_ = param.as_double(); }
      else if (param.get_name() == "bearing_friction") { bearing_friction_ = param.as_double(); }
    }
    if (body_needs_update) {
      update_flap_inertia();
      if (flap_->GetVisualModel()) { flap_->GetVisualModel()->Clear(); }
      flap_vis_shape_ = std::make_shared<ChVisualShapeBox>(
        flap_width_, kFlapVisDepth, flap_length_);
      flap_->AddVisualShape(flap_vis_shape_);
    }
  }
  // --- Publishing ---
  void publish_kinematics()
  {
    std_msgs::msg::Float64 pos_msg, vel_msg, accel_msg;
    pos_msg.data   = sim_position_;
    vel_msg.data   = sim_velocity_;
    accel_msg.data = sim_acceleration_;
    pos_pub_->publish(pos_msg);
    vel_pub_->publish(vel_msg);
    accel_pub_->publish(accel_msg);
  }
  void publish_joint_state()
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.push_back(joint_name_);
    js.position.push_back(sim_position_);
    js.velocity.push_back(sim_velocity_);
    js.effort.push_back(latest_torque_);
    joint_state_pub_->publish(js);
  }
  // --- Member variables ---
  // Mode
  std::string joint_name_{"motor_joint"};
  std::string joint_state_topic_{"/joint_states"};
  bool        sil_mode_{false};
  // Parameters
  double      rate_hz_{100.0};
  double      solver_rate_hz_{1000.0};
  double      publish_dt_{0.01};
  double      solver_dt_{0.001};
  int         substeps_{10};
  double      flap_length_{0.30};
  double      flap_width_{0.30};
  double      flap_mass_{0.5};
  double      joint_damping_{0.0};
  double      joint_stiffness_{0.712441};
  double      bearing_friction_{0.005};
  std::string effort_topic_{"/motor_effort_controller/commands"};
  bool        enable_vis_{false};
  bool        vis_active_{false};
  // Runtime state
  double latest_torque_;
  double sim_position_;
  double sim_velocity_;
  double sim_acceleration_;
  // Chrono objects
  std::unique_ptr<ChSystemNSC>                   sys_;
  std::shared_ptr<ChBody>                        ground_;
  std::shared_ptr<ChBody>                        flap_;
  std::shared_ptr<ChLinkMotorRotationTorque>     motor_link_;
  std::shared_ptr<ChFunctionSetpoint>            torque_fn_;
  std::shared_ptr<ChVisualShapeBox>              flap_vis_shape_;
#if defined(CHRONO_VSG) || defined(CHRONO_IRRLICHT)
  std::shared_ptr<ChVisualSystem>                vis_;
#endif
  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              accel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_state_pub_;
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