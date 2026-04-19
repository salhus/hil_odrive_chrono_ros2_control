// chrono_flap_node.cpp

// ChronoFlapNode — ROS 2 node that runs a Project Chrono multibody simulation of a motor
driving a rigid flap about a revolute joint.

// The node subscribes to the same effort/torque topic that the velocity_pid_node publishes to
// (`/motor_effort_controller/commands`, Float64MultiArray), extracts the first element as the
// applied torque, and steps a Chrono simulation at a configurable rate.  The resulting joint
// angle, angular velocity, and angular acceleration are published as Float64 topics under `~/`
// so they appear alongside the existing velocity_pid_node topics in PlotJuggler.

// Simulation model
// -----------------
//   - Ground body (fixed).
//   - Flap body: rigid bar of length L and mass m, approximated as a thin rod.
//     Inertia about one end: Ixx = Iyy = (1/3)*m*L^2, Izz ~ 0 (thin rod).
//   - ChLinkMotorRotationTorque connecting flap to ground at the root end.
//     A ChFunctionSetpoint is used to feed the current commanded torque on each timestep.
//   - Optional revolute joint damping applied as an external torque each tick.

// Runtime parameter reconfiguration
// -----------------------------------
//   - flap_length_m, flap_mass_kg, joint_damping -- reconfigurable at runtime via
//     rqt_reconfigure.  Changing flap_length_m or flap_mass_kg updates the Chrono body
//     (mass, inertia, CoM) in-place and resets flap state to the home position.
//   - rate_hz, effort_topic -- immutable; require node restart.

// Visualization
// -------------
//   Conditional 3D visualization via Chrono VSG (preferred) or Irrlicht (fallback).
//   Uses a manual simulation loop (not rclcpp::spin) to avoid segfaults from calling
//   VSG render functions inside ROS timer callbacks.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Project Chrono headers
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/functions/ChFunctionSetpoint.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/ChConfig.h"

// Visualization (optional -- conditional compilation)
#if defined(CHRONO_VSG)
#  include "chrono_vsg/ChVisualSystemVSG.h"
   using namespace chrono::vsg3d;
#elif defined(CHRONO_IRRLICHT)
#  include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
   using namespace chrono::irrlicht;
#endif

using namespace chrono;

// Visual shape constants
static constexpr double kFlapVisHeight   = 0.02;
static constexpr double kFlapVisDepth    = 0.01;
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
    // Parameters
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<double>("flap_length_m", 0.3);
    this->declare_parameter<double>("flap_mass_kg", 0.05);
    this->declare_parameter<double>("joint_damping", 0.001);
    this->declare_parameter<std::string>("effort_topic", "/motor_effort_controller/commands");

    rate_hz_       = this->get_parameter("rate_hz").as_double();
    flap_length_   = this->get_parameter("flap_length_m").as_double();
    flap_mass_     = this->get_parameter("flap_mass_kg").as_double();
    joint_damping_ = this->get_parameter("joint_damping").as_double();
    effort_topic_  = this->get_parameter("effort_topic").as_string();

    dt_ = (rate_hz_ > 0.0) ? (1.0 / rate_hz_) : 0.01;

    // Build Chrono system
    build_chrono_system();

    // ROS interfaces
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

    // Parameter callbacks
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
      "ChronoFlapNode started: rate=%.1f Hz, dt=%.4f s, flap=%.3f m / %.4f kg, damping=%.4f",
      rate_hz_, dt_, flap_length_, flap_mass_, joint_damping_);
  }

  // Run the simulation loop (called from main, NOT from rclcpp::spin).
  // This mirrors the pattern used by Chrono's own ROS demos (e.g. demo_ROS_hydraulic_crane).
  void run()
  {
    // Initialize visualization here (must be on the main thread)
    init_visualization();

    ChRealtimeStepTimer realtime_timer;

    while (rclcpp::ok()) {
      // Process ROS callbacks (subscriptions, parameter changes)
      rclcpp::spin_some(this->shared_from_this());

      // Render
#if defined(CHRONO_VSG) || defined(CHRONO_IRRLICHT)
      if (vis_) {
        if (!vis_->Run())
          break;
        vis_->BeginScene();
        vis_->Render();
        vis_->EndScene();
      }
#endif

      // Apply damping as an opposing torque added to the commanded torque
      const double damped_torque = latest_torque_ - joint_damping_ * sim_velocity_;
      torque_fn_->SetSetpoint(damped_torque, sys_->GetChTime());

      // Cache velocity before the step to compute acceleration
      const double vel_before = sim_velocity_;

      sys_->DoStepDynamics(dt_);

      // Read back kinematics from the motor link
      sim_position_ = motor_link_->GetMotorAngle();
      sim_velocity_ = motor_link_->GetMotorAngleDt();
      sim_acceleration_ = (sim_velocity_ - vel_before) / dt_;

      publish_kinematics();

      // Throttle to real time
      realtime_timer.Spin(dt_);
    }
  }

private:
  // Chrono system setup
  void build_chrono_system()
  {
    sys_ = std::make_unique<ChSystemNSC>();
    sys_->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Ground body (fixed)
    ground_ = std::make_shared<ChBody>();
    ground_->SetFixed(true);
    ground_->AddVisualShape(std::make_shared<ChVisualShapeBox>(
      kPivotMarkerSize, kPivotMarkerSize, kPivotMarkerSize));
    sys_->AddBody(ground_);

    // Flap body
    flap_ = std::make_shared<ChBody>();
    update_flap_inertia();

    flap_vis_shape_ = std::make_shared<ChVisualShapeBox>(flap_length_, kFlapVisHeight, kFlapVisDepth);
    flap_->AddVisualShape(flap_vis_shape_);
    sys_->AddBody(flap_);

    // Motor link
    motor_link_ = std::make_shared<ChLinkMotorRotationTorque>();
    torque_fn_  = std::make_shared<ChFunctionSetpoint>();
    motor_link_->SetTorqueFunction(torque_fn_);

    ChFrame<> pivot_frame(ChVector3d(0, 0, 0), QuatFromAngleX(0));
    motor_link_->Initialize(flap_, ground_, pivot_frame);
    sys_->AddLink(motor_link_);
  }

  // Visualization setup
  void init_visualization()
  {
#if defined(CHRONO_VSG)
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(sys_.get());
    vis->SetWindowTitle("Chrono Flap Simulation");
    vis->SetWindowSize(800, 600);
    vis->AddCamera(ChVector3d(0.15, -1.5, 0.3), ChVector3d(0.15, 0.0, 0.0));
    vis->Initialize();
    vis_ = vis;
    RCLCPP_INFO(this->get_logger(), "VSG visualization initialized.");
#elif defined(CHRONO_IRRLICHT)
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Chrono Flap Simulation");
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0.15, -1.5, 0.3), ChVector3d(0.15, 0.0, 0.0));
    vis->AddTypicalLights();
    vis->AttachSystem(sys_.get());
    vis_ = vis;
    RCLCPP_INFO(this->get_logger(), "Irrlicht visualization initialized.");
#else
    RCLCPP_INFO(this->get_logger(), "Running headless (no visualization).");
#endif
  }

  // Flap body inertia/CoM update
  void update_flap_inertia()
  {
    flap_->SetMass(flap_mass_);
    const double L      = flap_length_;
    const double m      = flap_mass_;
    const double I_perp = (1.0 / 3.0) * m * L * L;
    flap_->SetInertiaXX(ChVector3d(I_perp, I_perp, 1e-6 * I_perp));
    flap_->SetPos(ChVector3d(L / 2.0, 0.0, 0.0));
    flap_->SetPosDt(ChVector3d(0.0, 0.0, 0.0));
    flap_->SetAngVelParent(ChVector3d(0.0, 0.0, 0.0));
  }

  // Immutable parameters
  static inline const std::set<std::string> kImmutableParams = {"rate_hz", "effort_topic"};

  // Parameter validation
  rcl_interfaces::msg::SetParametersResult on_validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (kImmutableParams.count(param.get_name())) {
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() +
          "' cannot be changed at runtime (requires node restart).";
        return result;
      }

      if (param.get_name() == "flap_length_m" || param.get_name() == "flap_mass_kg") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            param.as_double() <= 0.0)
        {
          result.successful = false;
          result.reason = param.get_name() + " must be a positive double.";
          return result;
        }
      }

      if (param.get_name() == "joint_damping") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            param.as_double() < 0.0)
        {
          result.successful = false;
          result.reason = "joint_damping must be non-negative.";
          return result;
        }
      }
    }

    return result;
  }

  // Parameter apply
  void on_apply_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool body_needs_update = false;

    for (const auto & param : parameters) {
      if (param.get_name() == "flap_length_m") {
        flap_length_    = param.as_double();
        body_needs_update = true;
        RCLCPP_INFO(this->get_logger(), "Parameter updated: flap_length_m = %.4f", flap_length_);
      } else if (param.get_name() == "flap_mass_kg") {
        flap_mass_      = param.as_double();
        body_needs_update = true;
        RCLCPP_INFO(this->get_logger(), "Parameter updated: flap_mass_kg = %.4f", flap_mass_);
      } else if (param.get_name() == "joint_damping") {
        joint_damping_  = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter updated: joint_damping = %.4f", joint_damping_);
      }
    }

    if (body_needs_update) {
      update_flap_inertia();
      if (flap_->GetVisualModel()) {
        flap_->GetVisualModel()->Clear();
      }
      flap_vis_shape_ = std::make_shared<ChVisualShapeBox>(
        flap_length_, kFlapVisHeight, kFlapVisDepth);
      flap_->AddVisualShape(flap_vis_shape_);
      RCLCPP_INFO(
        this->get_logger(),
        "Flap body updated: length=%.4f m, mass=%.4f kg -- state reset to home position.",
        flap_length_, flap_mass_);
    }
  }

  // Publish kinematics
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

  // Parameters
  double      rate_hz_{100.0};
  double      dt_{0.01};
  double      flap_length_{0.3};
  double      flap_mass_{0.05};
  double      joint_damping_{0.001};
  std::string effort_topic_{"/motor_effort_controller/commands"};

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

  // Visualization
#if defined(CHRONO_VSG) || defined(CHRONO_IRRLICHT)
  std::shared_ptr<ChVisualSystem>                vis_;
#endif

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              accel_pub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

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