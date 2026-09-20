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

        position_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "position", 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
                if (std::isfinite(message->data)) {
                    current_position_rad_ = message->data;
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
                if (std::isfinite(message->data)) {
                    requested_position_rad_ = message->data;
                }
            });

        RCLCPP_INFO(get_logger(), "Angle controller node ready");
    }

private:
    double current_position_rad_{0.0};
    double current_velocity_rad_s_{0.0};
    double requested_position_rad_{0.0};
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
