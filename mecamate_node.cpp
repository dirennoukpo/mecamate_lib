/*
** mecamate_node.cpp for mecamate_lib [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecamate_lib
**
** Made by dirennoukpo
** Login   <diren.noukpo@epitech.eu>
**
** Started on  Fri May 15 19:00:50 2026 dirennoukpo
** Last update Sat May 15 19:00:52 2026 dirennoukpo
*/

/*
** mecamate_node.cpp
**
** ROS 2 node skeleton for Mecamate C++ driver
** Tested with ROS 2 Humble / Iron — C++17 required
**
** Build deps (package.xml):
**   rclcpp, std_msgs, sensor_msgs, geometry_msgs,
**   nav_msgs, trajectory_msgs, std_srvs
*/

#include <rclcpp/rclcpp.hpp>

// Publishers
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// Subscribers
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Services
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

// Driver
#include "Mecamate.hpp"

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
        declare_parameter("serial_port",  params_.serial_port);
        declare_parameter("car_type",     params_.car_type);
        declare_parameter("cmd_delay",    params_.cmd_delay);
        declare_parameter("debug",        params_.debug);
        declare_parameter("publish_rate", params_.publish_rate);

        get_parameter("serial_port",  params_.serial_port);
        get_parameter("car_type",     params_.car_type);
        get_parameter("cmd_delay",    params_.cmd_delay);
        get_parameter("debug",        params_.debug);
        get_parameter("publish_rate", params_.publish_rate);

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

        // ── Publishers ────────────────────────────────────────────────────────
        pub_imu_     = create_publisher<sensor_msgs::msg::Imu>(
                            "mecamate/imu/data", 10);
        pub_rpy_     = create_publisher<geometry_msgs::msg::Vector3Stamped>(
                            "mecamate/imu/rpy", 10);
        pub_mag_     = create_publisher<sensor_msgs::msg::MagneticField>(
                            "mecamate/imu/mag", 10);
        pub_odom_    = create_publisher<nav_msgs::msg::Odometry>(
                            "mecamate/odom", 10);
        pub_battery_ = create_publisher<sensor_msgs::msg::BatteryState>(
                            "mecamate/battery", 10);
        pub_enc_     = create_publisher<std_msgs::msg::Int32MultiArray>(
                            "mecamate/encoders", 10);

        // ── Subscribers ───────────────────────────────────────────────────────
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "mecamate/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                robot_->set_car_motion(msg->linear.x,
                                       msg->linear.y,
                                       msg->angular.z);
            });

        sub_motors_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "mecamate/motors/cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() < 4) {
                    RCLCPP_WARN(get_logger(),
                                "motors/cmd expects 4 values, got %zu", msg->data.size());
                    return;
                }
                robot_->set_motor(msg->data[0], msg->data[1],
                                  msg->data[2], msg->data[3]);
            });

        sub_pwm_servos_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "mecamate/pwm_servos/cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() < 4) {
                    RCLCPP_WARN(get_logger(),
                                "pwm_servos/cmd expects 4 values, got %zu", msg->data.size());
                    return;
                }
                robot_->set_pwm_servo_all(
                    static_cast<int>(msg->data[0]),
                    static_cast<int>(msg->data[1]),
                    static_cast<int>(msg->data[2]),
                    static_cast<int>(msg->data[3]));
            });

        sub_leds_ = create_subscription<std_msgs::msg::ColorRGBA>(
            "mecamate/leds/color", 10,
            [this](std_msgs::msg::ColorRGBA::ConstSharedPtr msg) {
                // led_id=0 → all LEDs; scale [0,1]→[0,255]
                robot_->set_colorful_lamps(
                    0,
                    static_cast<int>(msg->r * 255),
                    static_cast<int>(msg->g * 255),
                    static_cast<int>(msg->b * 255));
            });

        sub_arm_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "mecamate/arm/joint_cmd", 10,
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
                if (msg->points.empty()) return;
                const auto& pt = msg->points.front();
                if (pt.positions.size() < 6) {
                    RCLCPP_WARN(get_logger(),
                                "arm/joint_cmd: need 6 joint positions, got %zu",
                                pt.positions.size());
                    return;
                }
                // run_time from time_from_start (ms), capped at 2000 ms
                int run_time = static_cast<int>(
                    rclcpp::Duration(pt.time_from_start).seconds() * 1000.0);
                run_time = std::max(0, std::min(2000, run_time));

                std::vector<int> angles(6);
                for (int i = 0; i < 6; ++i)
                    angles[i] = static_cast<int>(pt.positions[i]);

                robot_->set_uart_servo_angle_array(angles, run_time);
            });

        sub_akm_ = create_subscription<std_msgs::msg::Int32>(
            "mecamate/akm/steering", 10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
                robot_->set_akm_steering_angle(msg->data, /*ctrl_car=*/true);
            });

        // ── Services ──────────────────────────────────────────────────────────
        srv_reset_flash_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/reset_flash",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
                   std_srvs::srv::Trigger::Response::SharedPtr      res) {
                robot_->reset_flash_value();
                res->success = true;
                res->message = "Flash reset sent";
            });

        srv_reset_car_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/reset_car",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
                   std_srvs::srv::Trigger::Response::SharedPtr      res) {
                robot_->reset_car_state();
                res->success = true;
                res->message = "Car state reset sent";
            });

        srv_beep_ = create_service<std_srvs::srv::Trigger>(
            "mecamate/beep",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
                   std_srvs::srv::Trigger::Response::SharedPtr      res) {
                robot_->set_beep(200);  // 200 ms beep — ajuste si besoin
                res->success = true;
                res->message = "Beep sent";
            });

        // ── Publish timer ─────────────────────────────────────────────────────
        auto period = std::chrono::duration<double>(1.0 / params_.publish_rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MecamateNode::publishAll, this));

        RCLCPP_INFO(get_logger(),
                    "mecamate_node ready — port=%s  car_type=%d  rate=%.0f Hz",
                    params_.serial_port.c_str(), params_.car_type, params_.publish_rate);
    }

private:
    // ── Publish loop ──────────────────────────────────────────────────────────
    void publishAll()
    {
        auto stamp = now();
        publishImu(stamp);
        publishRpy(stamp);
        publishMag(stamp);
        publishOdom(stamp);
        publishBattery(stamp);
        publishEncoders(stamp);
    }

    // ── sensor_msgs/Imu ───────────────────────────────────────────────────────
    void publishImu(const rclcpp::Time& stamp)
    {
        double ax, ay, az, gx, gy, gz;
        robot_->get_accelerometer_data(ax, ay, az);
        robot_->get_gyroscope_data(gx, gy, gz);

        sensor_msgs::msg::Imu msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        // Orientation non fournie par le driver → covariance -1 = unknown
        msg.orientation_covariance[0] = -1.0;

        // TODO: renseigne les covariances réelles si connues
        for (auto& v : msg.linear_acceleration_covariance) v = 0.0;
        for (auto& v : msg.angular_velocity_covariance)    v = 0.0;

        pub_imu_->publish(msg);
    }

    // ── geometry_msgs/Vector3Stamped (roll/pitch/yaw en degrés) ──────────────
    void publishRpy(const rclcpp::Time& stamp)
    {
        double roll, pitch, yaw;
        robot_->get_imu_attitude_data(roll, pitch, yaw, /*to_angle=*/true);

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "imu_link";
        msg.vector.x = roll;
        msg.vector.y = pitch;
        msg.vector.z = yaw;

        pub_rpy_->publish(msg);
    }

    // ── sensor_msgs/MagneticField ─────────────────────────────────────────────
    void publishMag(const rclcpp::Time& stamp)
    {
        double mx, my, mz;
        robot_->get_magnetometer_data(mx, my, mz);

        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "imu_link";
        msg.magnetic_field.x = mx;
        msg.magnetic_field.y = my;
        msg.magnetic_field.z = mz;
        msg.magnetic_field_covariance[0] = -1.0;  // unknown

        pub_mag_->publish(msg);
    }

    // ── nav_msgs/Odometry (vitesses seulement, pas de pose intégrée) ─────────
    void publishOdom(const rclcpp::Time& stamp)
    {
        double vx, vy, vz;
        robot_->get_motion_data(vx, vy, vz);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "odom";
        msg.child_frame_id  = "base_link";

        msg.twist.twist.linear.x  = vx;
        msg.twist.twist.linear.y  = vy;
        msg.twist.twist.angular.z = vz;

        // TODO: intègre la pose si nécessaire (tf2 broadcaster)

        pub_odom_->publish(msg);
    }

    // ── sensor_msgs/BatteryState ──────────────────────────────────────────────
    void publishBattery(const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = stamp;
        msg.voltage      = static_cast<float>(robot_->get_battery_voltage());
        msg.present      = true;
        // Champs non fournis par le driver
        msg.current      = std::numeric_limits<float>::quiet_NaN();
        msg.charge       = std::numeric_limits<float>::quiet_NaN();
        msg.capacity     = std::numeric_limits<float>::quiet_NaN();
        msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
        msg.percentage   = std::numeric_limits<float>::quiet_NaN();
        msg.power_supply_status     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

        pub_battery_->publish(msg);
    }

    // ── std_msgs/Int32MultiArray (4 encodeurs) ────────────────────────────────
    void publishEncoders(const rclcpp::Time& /*stamp*/)
    {
        int m1, m2, m3, m4;
        robot_->get_motor_encoder(m1, m2, m3, m4);

        std_msgs::msg::Int32MultiArray msg;
        msg.data = {m1, m2, m3, m4};

        pub_enc_->publish(msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    NodeParams params_;
    std::unique_ptr<Mecamate> robot_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr    pub_mag_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr     pub_battery_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr     pub_enc_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr              sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr       sub_motors_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr       sub_pwm_servos_;
    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr               sub_leds_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr  sub_arm_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                   sub_akm_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_flash_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_car_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_beep_;

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
        rclcpp::spin(std::make_shared<MecamateNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}