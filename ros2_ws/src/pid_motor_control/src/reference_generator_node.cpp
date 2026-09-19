#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pid_motor_control {

class ReferenceGeneratorNode : public rclcpp::Node {
public:
    ReferenceGeneratorNode()
        : Node("reference_generator") {
        signal_type_ = declare_parameter<std::string>("signal_type", "step");
        const double publish_frequency_hz =
            declare_parameter<double>("publish_frequency_hz", 100.0);
        amplitude_ = declare_parameter<double>("amplitude", 8.0);
        offset_ = declare_parameter<double>("offset", 0.0);
        step_time_s_ = declare_parameter<double>("step_time_s", 0.5);
        signal_frequency_hz_ = declare_parameter<double>("signal_frequency_hz", 0.2);

        if (signal_type_ != "step" && signal_type_ != "sine" && signal_type_ != "square") {
            throw std::invalid_argument("signal_type must be step, sine, or square");
        }
        if (!std::isfinite(publish_frequency_hz) || publish_frequency_hz <= 0.0
            || !std::isfinite(amplitude_) || !std::isfinite(offset_) || !std::isfinite(step_time_s_)
            || step_time_s_ < 0.0 || !std::isfinite(signal_frequency_hz_)
            || signal_frequency_hz_ < 0.0) {
            throw std::invalid_argument("reference generator parameters are invalid");
        }

        publisher_ = create_publisher<std_msgs::msg::Float64>("target_velocity", 10);
        start_time_ = now();
        const auto period = std::chrono::duration<double>(1.0 / publish_frequency_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ReferenceGeneratorNode::publish_reference, this));
        RCLCPP_INFO(get_logger(), "Velocity reference ready: signal=%s", signal_type_.c_str());
    }

private:
    void publish_reference() {
        const double elapsed_s = (now() - start_time_).seconds();
        constexpr double two_pi = 6.28318530717958647692;
        double value = offset_;
        if (signal_type_ == "step") {
            value += elapsed_s >= step_time_s_ ? amplitude_ : 0.0;
        } else if (signal_type_ == "sine") {
            value += amplitude_ * std::sin(two_pi * signal_frequency_hz_ * elapsed_s);
        } else {
            value += amplitude_
                   * (std::sin(two_pi * signal_frequency_hz_ * elapsed_s) >= 0.0 ? 1.0 : -1.0);
        }

        std_msgs::msg::Float64 message;
        message.data = value;
        publisher_->publish(message);
    }

    std::string signal_type_;
    double amplitude_{8.0};
    double offset_{0.0};
    double step_time_s_{0.5};
    double signal_frequency_hz_{0.2};
    rclcpp::Time start_time_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_motor_control::ReferenceGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
