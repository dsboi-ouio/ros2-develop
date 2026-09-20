#include <cmath>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pid_motor_control {

class AnglePidControllerNode : public rclcpp::Node {
public:
    AnglePidControllerNode()
        : Node("angle_pid_controller") {
        requested_position_rad_ =
            declare_parameter<double>("initial_target_position_rad", 1.5707963267948966);
        if (!std::isfinite(requested_position_rad_)) {
            throw std::invalid_argument("initial target position must be finite");
        }

        planned_position_publisher_ =
            create_publisher<std_msgs::msg::Float64>("planned_position", 10);

        position_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "position", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (std::isfinite(message->data)) {
                    current_position_rad_ = message->data;
                    if (target_pending_) {
                        plan_target();
                    }
                }
            });
        velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "velocity", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (std::isfinite(message->data)) {
                    current_velocity_rad_s_ = message->data;
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

        RCLCPP_INFO(get_logger(), "Angle controller node ready");
    }

private:
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
        std_msgs::msg::Float64 message;
        message.data = planned_position_rad_;
        planned_position_publisher_->publish(message);
        RCLCPP_INFO(
            get_logger(), "Angle target: current=%.3f, requested=%.3f, planned=%.3f",
            current_position_rad_, requested_position_rad_, planned_position_rad_);
    }

    double current_position_rad_{0.0};
    double current_velocity_rad_s_{0.0};
    double requested_position_rad_{0.0};
    double planned_position_rad_{0.0};
    bool target_pending_{true};
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr planned_position_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription_;
};

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_motor_control::AnglePidControllerNode>());
    rclcpp::shutdown();
    return 0;
}
