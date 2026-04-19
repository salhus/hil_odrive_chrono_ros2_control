// chrono_flap_node.cpp
//
// ChronoFlapNode — ROS 2 node that runs a Project Chrono multibody simulation of a motor
// driving a rigid flap about a revolute joint.
//
// The node subscribes to the same effort/torque topic that the velocity_pid_node publishes to
// (`/motor_effort_controller/commands`, Float64MultiArray), extracts the first element as the
// applied torque, and steps a Chrono simulation at a configurable rate.  The resulting joint
// angle, angular velocity, and angular acceleration are published as Float64 topics under `~/`
// so they appear alongside the existing velocity_pid_node topics in PlotJuggler.
//
// Simulation model
// ─────────────────
//   • Ground body (fixed).
//   • Flap body: rigid bar of length L and mass m, approximated as a thin rod.
//     Inertia about one end: Ixx = Iyy = (1/3)·m·L², Izz ≈ 0 (thin rod).
//   • ChLinkMotorRotationTorque connecting flap to ground at the root end.
//     A ChFunctionSetpoint is used to feed the current commanded torque on each timestep.
//   • Optional revolute joint damping applied as an external torque each tick.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Project Chrono headers
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/functions/ChFunctionSetpoint.h"

using namespace chrono;

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
    // ── Parameters ────────────────────────────────────────────────────────────────────────────
    // amplitude_rad_s / omega_rad_s mirror velocity_pid_node parameters; stored for reference
    // and future open-loop feed-forward use.
    this->declare_parameter<double>("amplitude_rad_s", 0.0);
    this->declare_parameter<double>("omega_rad_s", 0.0);
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<double>("flap_length_m", 0.3);
    this->declare_parameter<double>("flap_mass_kg", 0.05);
    this->declare_parameter<double>("joint_damping", 0.001);
    this->declare_parameter<std::string>("effort_topic", "/motor_effort_controller/commands");

    amplitude_rad_s_ = this->get_parameter("amplitude_rad_s").as_double();
    omega_rad_s_     = this->get_parameter("omega_rad_s").as_double();
    rate_hz_         = this->get_parameter("rate_hz").as_double();
    flap_length_     = this->get_parameter("flap_length_m").as_double();
    flap_mass_       = this->get_parameter("flap_mass_kg").as_double();
    joint_damping_   = this->get_parameter("joint_damping").as_double();
    effort_topic_    = this->get_parameter("effort_topic").as_string();

    dt_ = (rate_hz_ > 0.0) ? (1.0 / rate_hz_) : 0.01;

    // ── Build Chrono system ───────────────────────────────────────────────────────────────────
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

    pos_pub_   = this->create_publisher<std_msgs::msg::Float64>("~/sim_position",     10);
    vel_pub_   = this->create_publisher<std_msgs::msg::Float64>("~/sim_velocity",     10);
    accel_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/sim_acceleration", 10);

    // ── Simulation timer ──────────────────────────────────────────────────────────────────────
    using namespace std::chrono_literals;
    const auto period_ns = static_cast<int64_t>(dt_ * 1e9);
    timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(period_ns),
      std::bind(&ChronoFlapNode::simulation_step, this));

    RCLCPP_INFO(
      this->get_logger(),
      "ChronoFlapNode started: rate=%.1f Hz, dt=%.4f s, flap=%.3f m / %.4f kg, "
      "damping=%.4f, amplitude=%.4f rad/s, omega=%.4f rad/s",
      rate_hz_, dt_, flap_length_, flap_mass_, joint_damping_,
      amplitude_rad_s_, omega_rad_s_);
  }

  ~ChronoFlapNode()
  {
    if (timer_) {
      timer_->cancel();
    }
  }

private:
  // ── Chrono system setup ─────────────────────────────────────────────────────────────────────

  void build_chrono_system()
  {
    sys_ = std::make_unique<ChSystemNSC>();
    sys_->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Ground body (fixed)
    ground_ = std::make_shared<ChBody>();
    ground_->SetFixed(true);
    sys_->AddBody(ground_);

    // Flap body — thin rigid bar; pivot at origin, tip at (flap_length_, 0, 0)
    // CoM at mid-length: (flap_length_/2, 0, 0) relative to pivot
    flap_ = std::make_shared<ChBody>();
    flap_->SetMass(flap_mass_);

    // Inertia of a thin rod about one end:  I_perp = (1/3)*m*L^2
    const double L  = flap_length_;
    const double m  = flap_mass_;
    const double I_perp = (1.0 / 3.0) * m * L * L;
    // Iz (spin axis) is negligible for a thin rod; use a small positive value.
    flap_->SetInertiaXX(ChVector3d(I_perp, I_perp, 1e-6 * I_perp));

    // Place CoM at (L/2, 0, 0) so the pivot end is at world origin
    flap_->SetPos(ChVector3d(L / 2.0, 0.0, 0.0));
    sys_->AddBody(flap_);

    // Motor link: ChLinkMotorRotationTorque at world origin, rotation about Z axis
    motor_link_ = std::make_shared<ChLinkMotorRotationTorque>();
    torque_fn_  = std::make_shared<ChFunctionSetpoint>();
    motor_link_->SetTorqueFunction(torque_fn_);

    // Frame at the pivot (world origin), Z = rotation axis
    ChFrame<> pivot_frame(ChVector3d(0, 0, 0), QuatFromAngleX(0));
    motor_link_->Initialize(flap_, ground_, pivot_frame);
    sys_->AddLink(motor_link_);
  }

  // ── Timer callback ─────────────────────────────────────────────────────────────────────────

  void simulation_step()
  {
    // Apply damping as an opposing torque added to the commanded torque
    const double damped_torque = latest_torque_ - joint_damping_ * sim_velocity_;
    torque_fn_->SetSetpoint(damped_torque, sys_->GetChTime());

    // Cache velocity before the step to compute acceleration
    const double vel_before = sim_velocity_;

    sys_->DoStepDynamics(dt_);

    // Read back kinematics from the flap body
    // The motor angle / angular velocity are available from the motor link.
    sim_position_ = motor_link_->GetMotorAngle();
    sim_velocity_ = motor_link_->GetMotorAngleDt();

    // Finite-difference acceleration (Chrono exposes GetMotorAngleDt2() on some versions;
    // fall back to finite difference if unavailable)
    sim_acceleration_ = (sim_velocity_ - vel_before) / dt_;

    publish_kinematics();
  }

  // ── Publish ────────────────────────────────────────────────────────────────────────────────

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

  // ── Member variables ───────────────────────────────────────────────────────────────────────

  // Parameters
  double      amplitude_rad_s_{0.0};
  double      omega_rad_s_{0.0};
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

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr              accel_pub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChronoFlapNode>());
  rclcpp::shutdown();
  return 0;
}
