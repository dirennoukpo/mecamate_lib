/*
** mecamate_node.cpp for mecamate_lib [SSH: ROSMASTER-YAHBOOM]
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** ROS 2 node — full production implementation
** Tested with ROS 2 Humble / Iron — C++17 required
**
** Build deps (package.xml):
**   rclcpp, std_msgs, sensor_msgs, geometry_msgs,
**   nav_msgs, trajectory_msgs, std_srvs, tf2_ros, tf2_geometry_msgs
*/

// ─────────────────────────────────────────────────────────────────────────────
//  ROS 2 core
// ─────────────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  Publishers
// ─────────────────────────────────────────────────────────────────────────────
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  Subscribers
// ─────────────────────────────────────────────────────────────────────────────
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  Services
// ─────────────────────────────────────────────────────────────────────────────
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  TF2 (odom → base_link broadcast)
// ─────────────────────────────────────────────────────────────────────────────
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  Driver
// ─────────────────────────────────────────────────────────────────────────────
#include "Mecamate.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
//  Parameters (overridable from launch file or CLI)
// ─────────────────────────────────────────────────────────────────────────────
struct NodeParams {
    std::string serial_port  = "/dev/myserial";
    int         car_type     = 1;        // 1=X3, 2=X3_PLUS, 4=X1, 5=R2
    double      cmd_delay    = 0.002;    // seconds between serial writes
    bool        debug        = false;
    double      publish_rate = 50.0;     // Hz — auto-report publish loop
    bool        publish_tf   = true;     // broadcast odom→base_link tf
    std::string odom_frame   = "odom";
    std::string base_frame   = "base_link";
    std::string imu_frame    = "imu_link";
    // IMU covariances (diagonal only, rad²/s² and m²/s⁴)
    double gyro_cov          = 1e-4;
    double accel_cov         = 1e-2;
    double mag_cov           = 1e-4;
    // Arm control enable
    bool arm_enabled         = true;
    // Watchdog: stop motors if no cmd_vel received within this many seconds
    // Set 0.0 to disable.
    double cmd_vel_timeout   = 0.5;
    // Encoder ticks per revolution (used for joint_state publish)
    double ticks_per_rev     = 1625.0;
    // Wheel radius [m] — used for joint_state angular position integration
    double wheel_radius      = 0.045;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Pose integrator (dead-reckoning from velocity)
// ─────────────────────────────────────────────────────────────────────────────
struct OdomPose {
    double x{0}, y{0}, theta{0};

    void integrate(double vx, double vy, double vz, double dt) {
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);
        x     += (vx * cos_t - vy * sin_t) * dt;
        y     += (vx * sin_t + vy * cos_t) * dt;
        theta += vz * dt;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  MecamateNode
// ─────────────────────────────────────────────────────────────────────────────
class MecamateNode : public rclcpp::Node {
public:
    explicit MecamateNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("mecamate_node", options)
    {
        // ── Declare & read parameters ─────────────────────────────────────────
        declare_parameter("serial_port",    params_.serial_port);
        declare_parameter("car_type",       params_.car_type);
        declare_parameter("cmd_delay",      params_.cmd_delay);
        declare_parameter("debug",          params_.debug);
        declare_parameter("publish_rate",   params_.publish_rate);
        declare_parameter("publish_tf",     params_.publish_tf);
        declare_parameter("odom_frame",     params_.odom_frame);
        declare_parameter("base_frame",     params_.base_frame);
        declare_parameter("imu_frame",      params_.imu_frame);
        declare_parameter("gyro_cov",       params_.gyro_cov);
        declare_parameter("accel_cov",      params_.accel_cov);
        declare_parameter("mag_cov",        params_.mag_cov);
        declare_parameter("arm_enabled",    params_.arm_enabled);
        declare_parameter("cmd_vel_timeout", params_.cmd_vel_timeout);
        declare_parameter("ticks_per_rev",  params_.ticks_per_rev);
        declare_parameter("wheel_radius",   params_.wheel_radius);

        get_parameter("serial_port",    params_.serial_port);
        get_parameter("car_type",       params_.car_type);
        get_parameter("cmd_delay",      params_.cmd_delay);
        get_parameter("debug",          params_.debug);
        get_parameter("publish_rate",   params_.publish_rate);
        get_parameter("publish_tf",     params_.publish_tf);
        get_parameter("odom_frame",     params_.odom_frame);
        get_parameter("base_frame",     params_.base_frame);
        get_parameter("imu_frame",      params_.imu_frame);
        get_parameter("gyro_cov",       params_.gyro_cov);
        get_parameter("accel_cov",      params_.accel_cov);
        get_parameter("mag_cov",        params_.mag_cov);
        get_parameter("arm_enabled",    params_.arm_enabled);
        get_parameter("cmd_vel_timeout", params_.cmd_vel_timeout);
        get_parameter("ticks_per_rev",  params_.ticks_per_rev);
        get_parameter("wheel_radius",   params_.wheel_radius);

        // ── Open driver ───────────────────────────────────────────────────────
        try {
            robot_ = std::make_unique<Mecamate>(
                params_.car_type,
                params_.serial_port,
                params_.cmd_delay,
                params_.debug);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Failed to open Mecamate: %s", e.what());
            throw;
        }

        robot_->create_receive_threading();
        robot_->set_auto_report_state(true);
        robot_->set_uart_servo_ctrl_enable(params_.arm_enabled);

        // ── TF broadcaster ────────────────────────────────────────────────────
        if (params_.publish_tf) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // ── Publishers ────────────────────────────────────────────────────────
        pub_imu_     = create_publisher<sensor_msgs::msg::Imu>(
                            "mecamate/imu/data",    rclcpp::SensorDataQoS());
        pub_rpy_     = create_publisher<geometry_msgs::msg::Vector3Stamped>(
                            "mecamate/imu/rpy",     rclcpp::SensorDataQoS());
        pub_mag_     = create_publisher<sensor_msgs::msg::MagneticField>(
                            "mecamate/imu/mag",     rclcpp::SensorDataQoS());
        pub_odom_    = create_publisher<nav_msgs::msg::Odometry>(
                            "mecamate/odom",        rclcpp::SensorDataQoS());
        pub_battery_ = create_publisher<sensor_msgs::msg::BatteryState>(
                            "mecamate/battery",     10);
        pub_enc_     = create_publisher<std_msgs::msg::Int32MultiArray>(
                            "mecamate/encoders",    rclcpp::SensorDataQoS());
        pub_joint_   = create_publisher<sensor_msgs::msg::JointState>(
                            "mecamate/joint_states", rclcpp::SensorDataQoS());
        pub_vel_     = create_publisher<geometry_msgs::msg::TwistStamped>(
                            "mecamate/velocity",    rclcpp::SensorDataQoS());

        // ── Subscribers ───────────────────────────────────────────────────────

        // cmd_vel — main motion command
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "mecamate/cmd_vel", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                last_cmd_vel_time_ = now();
                motor_stopped_     = false;
                robot_->set_car_motion(msg->linear.x,
                                       msg->linear.y,
                                       msg->angular.z);
            });

        // raw motor speeds [−100..100] × 4
        sub_motors_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "mecamate/motors/cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() < 4) {
                    RCLCPP_WARN(get_logger(),
                        "motors/cmd: need 4 values, got %zu", msg->data.size());
                    return;
                }
                robot_->set_motor(msg->data[0], msg->data[1],
                                  msg->data[2], msg->data[3]);
            });

        // PWM servo positions [0..180] × 4
        sub_pwm_servos_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "mecamate/pwm_servos/cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() < 4) {
                    RCLCPP_WARN(get_logger(),
                        "pwm_servos/cmd: need 4 values, got %zu", msg->data.size());
                    return;
                }
                robot_->set_pwm_servo_all(
                    static_cast<int>(msg->data[0]),
                    static_cast<int>(msg->data[1]),
                    static_cast<int>(msg->data[2]),
                    static_cast<int>(msg->data[3]));
            });

        // LED color (ColorRGBA, values [0..1])
        sub_leds_ = create_subscription<std_msgs::msg::ColorRGBA>(
            "mecamate/leds/color", 10,
            [this](std_msgs::msg::ColorRGBA::ConstSharedPtr msg) {
                robot_->set_colorful_lamps(
                    0,
                    static_cast<int>(msg->r * 255),
                    static_cast<int>(msg->g * 255),
                    static_cast<int>(msg->b * 255));
            });

        // Arm — 6-DOF joint trajectory (positions in degrees)
        sub_arm_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "mecamate/arm/joint_cmd", 10,
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
                if (msg->points.empty()) return;
                const auto& pt = msg->points.front();
                if (pt.positions.size() < 6) {
                    RCLCPP_WARN(get_logger(),
                        "arm/joint_cmd: need 6 positions, got %zu",
                        pt.positions.size());
                    return;
                }
                // time_from_start → ms, clamped [0..2000]
                int run_time = static_cast<int>(
                    rclcpp::Duration(pt.time_from_start).seconds() * 1000.0);
                run_time = std::max(0, std::min(2000, run_time));

                std::vector<int> angles(6);
                for (int i = 0; i < 6; ++i)
                    angles[i] = static_cast<int>(pt.positions[i]);

                robot_->set_uart_servo_angle_array(angles, run_time);
            });

        // Ackermann steering angle [−45..45] degrees
        sub_akm_ = create_subscription<std_msgs::msg::Int32>(
            "mecamate/akm/steering", 10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
                robot_->set_akm_steering_angle(msg->data, /*ctrl_car=*/true);
            });

        // Arm enable/disable
        sub_arm_enable_ = create_subscription<std_msgs::msg::Bool>(
            "mecamate/arm/enable", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                robot_->set_uart_servo_ctrl_enable(msg->data);
                RCLCPP_INFO(get_logger(), "Arm ctrl %s",
                            msg->data ? "ENABLED" : "DISABLED");
            });

        // ── Services ──────────────────────────────────────────────────────────
        srv_reset_flash_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/reset_flash",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                robot_->reset_flash_value();
                res->success = true;
                res->message = "Flash reset sent";
            });

        srv_reset_car_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/reset_car",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                robot_->reset_car_state();
                res->success = true;
                res->message = "Car state reset sent";
            });

        srv_beep_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/beep",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                robot_->set_beep(200);
                res->success = true;
                res->message = "Beep sent (200 ms)";
            });

        srv_stop_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/stop",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                stopMotors();
                res->success = true;
                res->message = "Motors stopped";
            });

        srv_clear_odom_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/clear_odom",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                pose_ = OdomPose{};
                res->success = true;
                res->message = "Odometry pose reset to zero";
            });

        // Dynamic parameter callback
        param_cb_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                return onSetParameters(params);
            });

        // ── Publish timer ─────────────────────────────────────────────────────
        auto period = std::chrono::duration<double>(1.0 / params_.publish_rate);
        last_publish_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MecamateNode::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "mecamate_node ready\n"
            "  port=%s  car_type=%d  rate=%.0f Hz\n"
            "  publish_tf=%d  cmd_vel_timeout=%.2f s",
            params_.serial_port.c_str(), params_.car_type,
            params_.publish_rate, params_.publish_tf,
            params_.cmd_vel_timeout);
    }

private:
    // ─────────────────────────────────────────────────────────────────────────
    //  Timer callback — runs at publish_rate Hz
    // ─────────────────────────────────────────────────────────────────────────
    void timerCallback()
    {
        auto stamp = now();

        // ── Compute dt for odometry integration ───────────────────────────────
        double dt = (stamp - last_publish_time_).seconds();
        last_publish_time_ = stamp;
        if (dt <= 0.0 || dt > 1.0) dt = 1.0 / params_.publish_rate;

        // ── Watchdog: stop motors if cmd_vel is stale ─────────────────────────
        checkCmdVelTimeout(stamp);

        // ── Integrate pose ────────────────────────────────────────────────────
        double vx = 0, vy = 0, vz = 0;
        robot_->get_motion_data(vx, vy, vz);
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            pose_.integrate(vx, vy, vz, dt);
        }

        // ── Publish all topics ─────────────────────────────────────────────────
        publishImu(stamp);
        publishRpy(stamp);
        publishMag(stamp);
        publishOdom(stamp, vx, vy, vz);
        publishBattery(stamp);
        publishEncoders(stamp);
        publishJointStates(stamp);
        publishVelocity(stamp, vx, vy, vz);

        // ── TF broadcast ───────────────────────────────────────────────────────
        if (params_.publish_tf && tf_broadcaster_) {
            broadcastOdomTf(stamp);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Watchdog
    // ─────────────────────────────────────────────────────────────────────────
    void checkCmdVelTimeout(const rclcpp::Time& now_t)
    {
        if (params_.cmd_vel_timeout <= 0.0) return;
        if (motor_stopped_) return;
        if ((now_t - last_cmd_vel_time_).seconds() > params_.cmd_vel_timeout) {
            stopMotors();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "cmd_vel timeout — motors stopped");
        }
    }

    void stopMotors()
    {
        robot_->set_car_motion(0.0, 0.0, 0.0);
        motor_stopped_ = true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  sensor_msgs/Imu
    // ─────────────────────────────────────────────────────────────────────────
    void publishImu(const rclcpp::Time& stamp)
    {
        double ax, ay, az, gx, gy, gz;
        double roll, pitch, yaw;
        robot_->get_accelerometer_data(ax, ay, az);
        robot_->get_gyroscope_data(gx, gy, gz);
        // Orientation from IMU attitude (radians)
        robot_->get_imu_attitude_data(roll, pitch, yaw, /*to_angle=*/false);

        sensor_msgs::msg::Imu msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.imu_frame;

        // Convert Euler → quaternion
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        // Orientation covariance — diagonal, estimate ±2° → ~(0.035 rad)²
        msg.orientation_covariance.fill(0.0);
        msg.orientation_covariance[0] = 1.2e-3;
        msg.orientation_covariance[4] = 1.2e-3;
        msg.orientation_covariance[8] = 1.2e-3;

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        msg.linear_acceleration_covariance.fill(0.0);
        msg.linear_acceleration_covariance[0] = params_.accel_cov;
        msg.linear_acceleration_covariance[4] = params_.accel_cov;
        msg.linear_acceleration_covariance[8] = params_.accel_cov;

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;
        msg.angular_velocity_covariance.fill(0.0);
        msg.angular_velocity_covariance[0] = params_.gyro_cov;
        msg.angular_velocity_covariance[4] = params_.gyro_cov;
        msg.angular_velocity_covariance[8] = params_.gyro_cov;

        pub_imu_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  geometry_msgs/Vector3Stamped (RPY en degrés)
    // ─────────────────────────────────────────────────────────────────────────
    void publishRpy(const rclcpp::Time& stamp)
    {
        double roll, pitch, yaw;
        robot_->get_imu_attitude_data(roll, pitch, yaw, /*to_angle=*/true);

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.imu_frame;
        msg.vector.x = roll;
        msg.vector.y = pitch;
        msg.vector.z = yaw;

        pub_rpy_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  sensor_msgs/MagneticField
    // ─────────────────────────────────────────────────────────────────────────
    void publishMag(const rclcpp::Time& stamp)
    {
        double mx, my, mz;
        robot_->get_magnetometer_data(mx, my, mz);

        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.imu_frame;
        msg.magnetic_field.x = mx;
        msg.magnetic_field.y = my;
        msg.magnetic_field.z = mz;
        msg.magnetic_field_covariance.fill(0.0);
        msg.magnetic_field_covariance[0] = params_.mag_cov;
        msg.magnetic_field_covariance[4] = params_.mag_cov;
        msg.magnetic_field_covariance[8] = params_.mag_cov;

        pub_mag_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  nav_msgs/Odometry — pose intégrée + vitesses
    // ─────────────────────────────────────────────────────────────────────────
    void publishOdom(const rclcpp::Time& stamp,
                     double vx, double vy, double vz)
    {
        OdomPose p;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            p = pose_;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.theta);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.odom_frame;
        msg.child_frame_id  = params_.base_frame;

        // Pose
        msg.pose.pose.position.x    = p.x;
        msg.pose.pose.position.y    = p.y;
        msg.pose.pose.position.z    = 0.0;
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        // Pose covariance (diagonal) — dead-reckoning drifts fast
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0]  = 1e-2;  // x
        msg.pose.covariance[7]  = 1e-2;  // y
        msg.pose.covariance[35] = 1e-2;  // yaw

        // Twist (in base_link frame)
        msg.twist.twist.linear.x  = vx;
        msg.twist.twist.linear.y  = vy;
        msg.twist.twist.angular.z = vz;

        msg.twist.covariance.fill(0.0);
        msg.twist.covariance[0]  = 1e-3;
        msg.twist.covariance[7]  = 1e-3;
        msg.twist.covariance[35] = 1e-3;

        pub_odom_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  TF2 broadcast: odom → base_link
    // ─────────────────────────────────────────────────────────────────────────
    void broadcastOdomTf(const rclcpp::Time& stamp)
    {
        OdomPose p;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            p = pose_;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.theta);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = params_.odom_frame;
        tf.child_frame_id  = params_.base_frame;

        tf.transform.translation.x = p.x;
        tf.transform.translation.y = p.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x    = q.x();
        tf.transform.rotation.y    = q.y();
        tf.transform.rotation.z    = q.z();
        tf.transform.rotation.w    = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  sensor_msgs/BatteryState
    // ─────────────────────────────────────────────────────────────────────────
    void publishBattery(const rclcpp::Time& stamp)
    {
        constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = stamp;
        msg.voltage      = static_cast<float>(robot_->get_battery_voltage());
        msg.present      = true;
        msg.current      = kNaN;
        msg.charge       = kNaN;
        msg.capacity     = kNaN;
        msg.design_capacity = kNaN;
        msg.percentage   = kNaN;
        msg.power_supply_status     =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health     =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_technology =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

        pub_battery_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  std_msgs/Int32MultiArray — 4 encoder raw counts
    // ─────────────────────────────────────────────────────────────────────────
    void publishEncoders(const rclcpp::Time& /*stamp*/)
    {
        int m1, m2, m3, m4;
        robot_->get_motor_encoder(m1, m2, m3, m4);

        std_msgs::msg::Int32MultiArray msg;
        msg.data = {m1, m2, m3, m4};
        pub_enc_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  sensor_msgs/JointState — wheel positions derived from encoders
    //
    //  This topic is consumed by ros2_control's JointStateBroadcaster or by
    //  robot_state_publisher when ros2_control is NOT used.
    //  joint names must match the URDF.
    // ─────────────────────────────────────────────────────────────────────────
    void publishJointStates(const rclcpp::Time& stamp)
    {
        int m1, m2, m3, m4;
        robot_->get_motor_encoder(m1, m2, m3, m4);

        // Convert encoder ticks → radians
        auto ticks2rad = [&](int ticks) -> double {
            return (static_cast<double>(ticks) / params_.ticks_per_rev)
                   * 2.0 * M_PI;
        };

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name     = {"wheel_fl_joint", "wheel_fr_joint",
                         "wheel_rl_joint", "wheel_rr_joint"};
        msg.position = {ticks2rad(m1), ticks2rad(m2),
                        ticks2rad(m3), ticks2rad(m4)};
        // velocity from motion report (vx shared, per-wheel not available)
        // If needed, differentiate encoder counts between calls.
        msg.velocity = {0.0, 0.0, 0.0, 0.0};
        msg.effort   = {};

        pub_joint_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  geometry_msgs/TwistStamped — stamped velocity (for ros2_control bridge)
    // ─────────────────────────────────────────────────────────────────────────
    void publishVelocity(const rclcpp::Time& stamp,
                         double vx, double vy, double vz)
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = params_.base_frame;
        msg.twist.linear.x  = vx;
        msg.twist.linear.y  = vy;
        msg.twist.angular.z = vz;
        pub_vel_->publish(msg);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Dynamic parameter update callback
    // ─────────────────────────────────────────────────────────────────────────
    rcl_interfaces::msg::SetParametersResult
    onSetParameters(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            if (p.get_name() == "publish_tf") {
                params_.publish_tf = p.as_bool();
            } else if (p.get_name() == "cmd_vel_timeout") {
                params_.cmd_vel_timeout = p.as_double();
            } else if (p.get_name() == "arm_enabled") {
                params_.arm_enabled = p.as_bool();
                robot_->set_uart_servo_ctrl_enable(params_.arm_enabled);
            } else if (p.get_name() == "gyro_cov") {
                params_.gyro_cov = p.as_double();
            } else if (p.get_name() == "accel_cov") {
                params_.accel_cov = p.as_double();
            } else if (p.get_name() == "mag_cov") {
                params_.mag_cov = p.as_double();
            }
        }
        return result;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Members
    // ─────────────────────────────────────────────────────────────────────────
    NodeParams params_;
    std::unique_ptr<Mecamate> robot_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Dead-reckoning pose
    OdomPose          pose_;
    std::mutex        pose_mutex_;
    rclcpp::Time      last_publish_time_;

    // cmd_vel watchdog
    rclcpp::Time      last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    bool              motor_stopped_{true};

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    pub_mag_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_battery_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr     pub_enc_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       pub_joint_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   pub_vel_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      sub_motors_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr      sub_pwm_servos_;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr              sub_leds_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_arm_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                  sub_akm_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                   sub_arm_enable_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_flash_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_car_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_beep_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_odom_;

    // Dynamic params handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<MecamateNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"),
                     "Unhandled exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}