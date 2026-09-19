#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

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

        RCLCPP_INFO(
            get_logger(), "Velocity PID ready: rate=%.1f Hz, torque_limit=%.2f Nm",
            control_frequency_hz, torque_limit_nm);
    }

private:
    double dt_{0.001};
    double feedback_timeout_s_{0.1};
    double target_velocity_rad_s_{0.0};

  std::unique_ptr<PidController> pid_;
};

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pid_motor_control::PidControllerNode>());
    rclcpp::shutdown();
    return 0;
}
