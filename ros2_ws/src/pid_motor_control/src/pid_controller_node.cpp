#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pid_motor_control/pid_controller.hpp"

namespace pid_motor_control {

class PidControllerNode : public rclcpp::Node {
public:
    PidControllerNode()
        : Node("pid_controller") {
        const double control_frequency_hz =
            declare_parameter<double>("control_frequency_hz", 1000.0);
        const double feedback_timeout_s = declare_parameter<double>("feedback_timeout_s", 0.1);
        const double torque_limit_nm = declare_parameter<double>("torque_limit_nm", 2.0);
        target_velocity_rad_s_ = declare_parameter<double>("initial_target_velocity_rad_s", 0.0);

        if (!std::isfinite(control_frequency_hz) || control_frequency_hz < 500.0) {
            throw std::invalid_argument("control_frequency_hz must be at least 500 Hz");
        }
        if (!std::isfinite(feedback_timeout_s) || feedback_timeout_s <= 0.0
            || !std::isfinite(torque_limit_nm) || torque_limit_nm <= 0.0
            || !std::isfinite(target_velocity_rad_s_)) {
            throw std::invalid_argument("controller parameters must be finite and valid");
        }

        dt_ = 1.0 / control_frequency_hz;
        feedback_timeout_s_ = feedback_timeout_s;
        pid_ = std::make_unique<PidController>(
            PidGains{
                declare_parameter<double>("pid.kp", 0.35), declare_parameter<double>("pid.ki", 2.0),
                declare_parameter<double>("pid.kd", 0.0005)},
            PidLimits{
                -torque_limit_nm, torque_limit_nm,
                declare_parameter<double>("pid.integral_min", -10.0),
                declare_parameter<double>("pid.integral_max", 10.0)});

    torque_publisher_ = create_publisher<std_msgs::msg::Float64>("torque_cmd", 10);

        velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "velocity", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (!std::isfinite(message->data)) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000, "Ignoring non-finite velocity feedback");
                    return;
                }
                current_velocity_rad_s_ = message->data;
                velocity_received_ = true;
                last_velocity_time_ = std::chrono::steady_clock::now();
            });
        target_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "target_velocity", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (std::isfinite(message->data)) {
                    target_velocity_rad_s_ = message->data;
                }
            });

        const auto period = std::chrono::duration<double>(dt_);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PidControllerNode::control_step, this));

        RCLCPP_INFO(
            get_logger(), "Velocity PID ready: rate=%.1f Hz, torque_limit=%.2f Nm",
            control_frequency_hz, torque_limit_nm);
    }

private:
    static void publish_value(
        const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& publisher, double value) {
        std_msgs::msg::Float64 message;
        message.data = value;
        publisher->publish(message);
    }

    void control_step() {
        if (!velocity_received_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double feedback_age_s =
            std::chrono::duration<double>(now - last_velocity_time_).count();
        if (feedback_age_s > feedback_timeout_s_) {
            publish_value(torque_publisher_, 0.0);
            if (!feedback_was_stale_) {
                pid_->reset();
                feedback_was_stale_ = true;
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Velocity feedback is stale; commanding zero torque");
            return;
        }
        feedback_was_stale_ = false;

    const double error = target_velocity_rad_s_ - current_velocity_rad_s_;
    const double torque = pid_->update_error(error, dt_);
    publish_value(torque_publisher_, torque);
    }

    double dt_{0.001};
    double feedback_timeout_s_{0.1};
    double current_velocity_rad_s_{0.0};
    double target_velocity_rad_s_{0.0};
    bool velocity_received_{false};
    bool feedback_was_stale_{false};
    std::chrono::steady_clock::time_point last_velocity_time_;

  std::unique_ptr<PidController> pid_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_motor_control::PidControllerNode>());
    rclcpp::shutdown();
    return 0;
}
