#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace motor_simulator
{

class TorqueSourceNode : public rclcpp::Node
{
public:
  TorqueSourceNode()
  : Node("torque_source"),
    publish_frequency_hz_(declare_parameter<double>("publish_frequency_hz", 100.0)),
    mode_(declare_parameter<std::string>("mode", "step")),
    amplitude_(declare_parameter<double>("amplitude", 0.2)),
    offset_(declare_parameter<double>("offset", 0.0)),
    step_time_s_(declare_parameter<double>("step_time_s", 0.5)),
    signal_frequency_hz_(declare_parameter<double>("signal_frequency_hz", 0.5))
  {
    if (publish_frequency_hz_ <= 0.0) {
      throw std::invalid_argument("publish_frequency_hz must be greater than zero");
    }
    if (mode_ != "step" && mode_ != "sine" && mode_ != "square") {
      throw std::invalid_argument("mode must be step, sine, or square");
    }

    publisher_ = create_publisher<std_msgs::msg::Float64>("torque_cmd", 10);
    start_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TorqueSourceNode::publish_torque, this));
    RCLCPP_INFO(get_logger(), "Torque source ready: mode=%s", mode_.c_str());
  }

private:
  void publish_torque()
  {
    const double elapsed_s = (now() - start_time_).seconds();
    double value = offset_;
    constexpr double two_pi = 6.28318530717958647692;

    if (mode_ == "step") {
      value += elapsed_s >= step_time_s_ ? amplitude_ : 0.0;
    } else if (mode_ == "sine") {
      value += amplitude_ * std::sin(two_pi * signal_frequency_hz_ * elapsed_s);
    } else {
      value += amplitude_ *
        (std::sin(two_pi * signal_frequency_hz_ * elapsed_s) >= 0.0 ? 1.0 : -1.0);
    }

    std_msgs::msg::Float64 message;
    message.data = value;
    publisher_->publish(message);
  }

  double publish_frequency_hz_;
  std::string mode_;
  double amplitude_;
  double offset_;
  double step_time_s_;
  double signal_frequency_hz_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motor_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_simulator::TorqueSourceNode>());
  rclcpp::shutdown();
  return 0;
}
