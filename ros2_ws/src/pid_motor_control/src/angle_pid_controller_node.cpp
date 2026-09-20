#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pid_motor_control/pid_controller.hpp"

namespace pid_motor_control {

class AnglePidControllerNode : public rclcpp::Node {
public:
    AnglePidControllerNode()
        : Node("angle_pid_controller") {
        const double control_frequency_hz =
            declare_parameter<double>("control_frequency_hz", 1000.0);
        const double velocity_limit_rad_s =
            declare_parameter<double>("velocity_limit_rad_s", 4.0);
        const double torque_limit_nm = declare_parameter<double>("torque_limit_nm", 2.0);
        requested_position_rad_ =
            declare_parameter<double>("initial_target_position_rad", 1.5707963267948966);
        if (!std::isfinite(control_frequency_hz) || control_frequency_hz < 500.0
            || !std::isfinite(velocity_limit_rad_s) || velocity_limit_rad_s <= 0.0
            || !std::isfinite(torque_limit_nm) || torque_limit_nm <= 0.0
            || !std::isfinite(requested_position_rad_)) {
            throw std::invalid_argument("angle controller parameters are invalid");
        }

        dt_ = 1.0 / control_frequency_hz;
        position_pid_ = std::make_unique<PidController>(
            PidGains{
                declare_parameter<double>("position_pid.kp", 1.5),
                declare_parameter<double>("position_pid.ki", 0.05),
                declare_parameter<double>("position_pid.kd", 0.05)},
            PidLimits{
                -velocity_limit_rad_s, velocity_limit_rad_s,
                declare_parameter<double>("position_pid.integral_min", -1.0),
                declare_parameter<double>("position_pid.integral_max", 1.0)});
        velocity_pid_ = std::make_unique<PidController>(
            PidGains{
                declare_parameter<double>("velocity_pid.kp", 0.35),
                declare_parameter<double>("velocity_pid.ki", 2.0),
                declare_parameter<double>("velocity_pid.kd", 0.0005)},
            PidLimits{
                -torque_limit_nm, torque_limit_nm,
                declare_parameter<double>("velocity_pid.integral_min", -10.0),
                declare_parameter<double>("velocity_pid.integral_max", 10.0)});

        torque_publisher_ = create_publisher<std_msgs::msg::Float64>("torque_cmd", 10);
        target_velocity_publisher_ =
            create_publisher<std_msgs::msg::Float64>("target_velocity", 10);
        planned_position_publisher_ =
            create_publisher<std_msgs::msg::Float64>("planned_position", 10);

        position_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "position", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (std::isfinite(message->data)) {
                    current_position_rad_ = message->data;
                    position_received_ = true;
                    if (target_pending_) {
                        plan_target();
                    }
                }
            });
        velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "velocity", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (std::isfinite(message->data)) {
                    current_velocity_rad_s_ = message->data;
                    velocity_received_ = true;
                }
            });
        target_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "target_position", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (!std::isfinite(message->data)) {
                    return;
                }
                constexpr double two_pi = 6.28318530717958647692;
                if (std::abs(std::remainder(message->data - requested_position_rad_, two_pi))
                    < 1e-9) {
                    return;
                }
                requested_position_rad_ = message->data;
                target_pending_ = true;
            });

        const auto period = std::chrono::duration<double>(dt_);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&AnglePidControllerNode::control_step, this));

        RCLCPP_INFO(get_logger(), "Angle PID ready: rate=%.1f Hz", control_frequency_hz);
    }

private:
    static void publish_value(
        const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher, double value) {
        std_msgs::msg::Float64 message;
        message.data = value;
        publisher->publish(message);
    }

    void plan_target() {
        constexpr double pi = 3.14159265358979323846;
        constexpr double two_pi = 2.0 * pi;
        const double target_angle = std::remainder(requested_position_rad_, two_pi);
        const double current_angle = std::remainder(current_position_rad_, two_pi);
        const double short_delta = std::remainder(target_angle - current_angle, two_pi);
        double travel = 0.0;
        if (std::abs(short_delta) >= pi - 1e-9) {
            travel = pi;
        } else if (std::abs(short_delta) > 1e-9) {
            travel = short_delta - std::copysign(two_pi, short_delta);
        }

        planned_position_rad_ = current_position_rad_ + travel;
        target_pending_ = false;
        position_pid_->reset();
        velocity_pid_->reset();
        publish_value(planned_position_publisher_, planned_position_rad_);
        RCLCPP_INFO(
            get_logger(), "Angle target: current=%.3f, requested=%.3f, planned=%.3f",
            current_position_rad_, requested_position_rad_, planned_position_rad_);
    }

    void control_step() {
        if (!position_received_ || !velocity_received_ || target_pending_) {
            return;
        }
        const double target_velocity_rad_s =
            position_pid_->update(planned_position_rad_, current_position_rad_, dt_);
        const double torque_nm =
            velocity_pid_->update(target_velocity_rad_s, current_velocity_rad_s_, dt_);
        publish_value(planned_position_publisher_, planned_position_rad_);
        publish_value(target_velocity_publisher_, target_velocity_rad_s);
        publish_value(torque_publisher_, torque_nm);
    }

    double dt_{0.001};
    double current_position_rad_{0.0};
    double current_velocity_rad_s_{0.0};
    double requested_position_rad_{0.0};
    double planned_position_rad_{0.0};
    bool position_received_{false};
    bool velocity_received_{false};
    bool target_pending_{true};
    std::unique_ptr<PidController> position_pid_;
    std::unique_ptr<PidController> velocity_pid_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr planned_position_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_motor_control::AnglePidControllerNode>());
    rclcpp::shutdown();
    return 0;
}
