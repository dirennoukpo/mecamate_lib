// mecamate_hardware.hpp
// ros2_control SystemInterface for Mecamate (mecanum / differential)
//
// Implements the hardware interface layer so that standard ros2_control
// controllers (diff_drive_controller, mecanum_drive_controller,
// joint_trajectory_controller …) can drive the robot directly.
//
// ── How it fits in ros2_control ─────────────────────────────────────────────
//
//   Controller Manager
//       │
//       ├─► diff_drive_controller  (or mecanum_drive_controller)
//       │       reads: odom, publishes: cmd_vel
//       │
//       └─► MecamateHardware  ← YOU ARE HERE
//               reads:  Mecamate encoder ticks  → state interfaces (pos / vel)
//               writes: Mecamate set_car_motion  ← command interfaces (vel)
//
// ── Interfaces exposed ────────────────────────────────────────────────────────
//   State   (per wheel joint): position [rad], velocity [rad/s]
//   Command (per wheel joint): velocity [rad/s]
//
// ── package.xml deps ──────────────────────────────────────────────────────────
//   hardware_interface, pluginlib, rclcpp, rclcpp_lifecycle
// ─────────────────────────────────────────────────────────────────────────────

#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "Mecamate.hpp"

namespace mecamate_ros2_control {

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

// ─────────────────────────────────────────────────────────────────────────────
//  MecamateHardware
// ─────────────────────────────────────────────────────────────────────────────
class MecamateHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MecamateHardware)

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&)          override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&)           override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)         override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)            override;

    // ── Interface export ──────────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ── Read / Write ──────────────────────────────────────────────────────────
    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // ── Parameters from URDF ros2_control block ───────────────────────────────
    std::string serial_port_    = "/dev/myserial";
    int         car_type_       = 1;
    double      cmd_delay_      = 0.002;
    bool        debug_          = false;
    double      ticks_per_rev_  = 1625.0;
    double      wheel_radius_   = 0.045;   // [m]
    double      wheel_sep_x_    = 0.14;    // half wheel-base front/back [m]
    double      wheel_sep_y_    = 0.12;    // half wheel-base left/right [m]

    // ── Driver ────────────────────────────────────────────────────────────────
    std::unique_ptr<Mecamate> robot_;

    // ── Joint state & command (FL, FR, RL, RR order) ─────────────────────────
    // 4 wheels × {position, velocity}
    static constexpr int N = 4;
    std::array<double, N> hw_pos_   {0.0, 0.0, 0.0, 0.0};  // [rad]
    std::array<double, N> hw_vel_   {0.0, 0.0, 0.0, 0.0};  // [rad/s]
    std::array<double, N> hw_cmd_   {0.0, 0.0, 0.0, 0.0};  // [rad/s]

    // Previous encoder ticks for velocity computation
    std::array<int, N>    prev_enc_ {0, 0, 0, 0};
    rclcpp::Time          prev_time_{0, 0, RCL_ROS_TIME};

    // ── Helpers ───────────────────────────────────────────────────────────────
    // rad/s → motor "speed" in the Mecamate protocol
    // Mecamate set_car_motion wants m/s and rad/s in the body frame.
    // We derive vx, vy, vz from wheel commands using mecanum kinematics.
    void computeBodyVelocity(double& vx, double& vy, double& vz) const;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  Implementation (header-only for a single TU — split to .cpp if you prefer)
// ═══════════════════════════════════════════════════════════════════════════════

inline CallbackReturn
MecamateHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // Read hardware parameters from the URDF <ros2_control> block
    auto get_param = [&](const std::string& key, const std::string& dflt) {
        return info_.hardware_parameters.count(key)
               ? info_.hardware_parameters.at(key) : dflt;
    };

    serial_port_   = get_param("serial_port",    "/dev/myserial");
    car_type_      = std::stoi(get_param("car_type",      "1"));
    cmd_delay_     = std::stod(get_param("cmd_delay",     "0.002"));
    debug_         = (get_param("debug", "false") == "true");
    ticks_per_rev_ = std::stod(get_param("ticks_per_rev", "1625"));
    wheel_radius_  = std::stod(get_param("wheel_radius",  "0.045"));
    wheel_sep_x_   = std::stod(get_param("wheel_sep_x",   "0.14"));
    wheel_sep_y_   = std::stod(get_param("wheel_sep_y",   "0.12"));

    // Validate joint count (must be 4)
    if (info_.joints.size() != 4) {
        RCLCPP_FATAL(rclcpp::get_logger("MecamateHardware"),
            "Expected 4 joints, got %zu", info_.joints.size());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MecamateHardware"),
        "on_init OK — port=%s  car_type=%d", serial_port_.c_str(), car_type_);
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecamateHardware::on_configure(const rclcpp_lifecycle::State&)
{
    try {
        robot_ = std::make_unique<Mecamate>(
            car_type_, serial_port_, cmd_delay_, debug_);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("MecamateHardware"),
            "Failed to open serial: %s", e.what());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("MecamateHardware"), "on_configure OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecamateHardware::on_activate(const rclcpp_lifecycle::State&)
{
    robot_->create_receive_threading();
    robot_->set_auto_report_state(true);

    prev_time_ = rclcpp::Clock().now();
    robot_->get_motor_encoder(prev_enc_[0], prev_enc_[1],
                               prev_enc_[2], prev_enc_[3]);

    // Initialise state interfaces to 0
    hw_pos_.fill(0.0);
    hw_vel_.fill(0.0);
    hw_cmd_.fill(0.0);

    RCLCPP_INFO(rclcpp::get_logger("MecamateHardware"), "on_activate OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecamateHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    // Safety: stop motors before going inactive
    if (robot_) robot_->set_car_motion(0.0, 0.0, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("MecamateHardware"), "on_deactivate OK");
    return CallbackReturn::SUCCESS;
}

inline CallbackReturn
MecamateHardware::on_cleanup(const rclcpp_lifecycle::State&)
{
    robot_.reset();
    RCLCPP_INFO(rclcpp::get_logger("MecamateHardware"), "on_cleanup OK");
    return CallbackReturn::SUCCESS;
}

inline std::vector<hardware_interface::StateInterface>
MecamateHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
        si.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
    }
    return si;
}

inline std::vector<hardware_interface::CommandInterface>
MecamateHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_cmd_[i]);
    }
    return ci;
}

// ── READ: encoder → position/velocity state interfaces ───────────────────────
inline return_type
MecamateHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    int enc[4];
    robot_->get_motor_encoder(enc[0], enc[1], enc[2], enc[3]);

    double dt = period.seconds();
    if (dt <= 0.0) dt = 0.02;

    for (int i = 0; i < N; ++i) {
        int delta     = enc[i] - prev_enc_[i];
        double d_rad  = (static_cast<double>(delta) / ticks_per_rev_) * 2.0 * M_PI;
        hw_pos_[i]   += d_rad;
        hw_vel_[i]    = d_rad / dt;
        prev_enc_[i]  = enc[i];
    }
    prev_time_ = time;
    return return_type::OK;
}

// ── WRITE: command interfaces → Mecamate body-frame motion ───────────────────
inline return_type
MecamateHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    double vx, vy, vz;
    computeBodyVelocity(vx, vy, vz);
    robot_->set_car_motion(vx, vy, vz);
    return return_type::OK;
}

// ── Mecanum inverse kinematics: wheel ω [rad/s] → body v [m/s, rad/s] ────────
// Wheel order: FL=0, FR=1, RL=2, RR=3
// Standard mecanum forward kinematics:
//   vx  = (ω0 + ω1 + ω2 + ω3) * r / 4
//   vy  = (-ω0 + ω1 + ω2 - ω3) * r / 4
//   vz  = (-ω0 + ω1 - ω2 + ω3) * r / (4*(lx+ly))
inline void
MecamateHardware::computeBodyVelocity(double& vx, double& vy, double& vz) const
{
    double r  = wheel_radius_;
    double lxy = wheel_sep_x_ + wheel_sep_y_;

    double w0 = hw_cmd_[0]; // FL
    double w1 = hw_cmd_[1]; // FR
    double w2 = hw_cmd_[2]; // RL
    double w3 = hw_cmd_[3]; // RR

    vx = r / 4.0 * ( w0 + w1 + w2 + w3);
    vy = r / 4.0 * (-w0 + w1 + w2 - w3);
    vz = r / (4.0 * lxy) * (-w0 + w1 - w2 + w3);
}

}  // namespace mecamate_ros2_control

// ─────────────────────────────────────────────────────────────────────────────
//  pluginlib export (put in mecamate_hardware.cpp, not the header)
// ─────────────────────────────────────────────────────────────────────────────
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(
//   mecamate_ros2_control::MecamateHardware,
//   hardware_interface::SystemInterface)
