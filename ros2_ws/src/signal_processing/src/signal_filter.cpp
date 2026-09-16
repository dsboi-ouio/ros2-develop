#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "signal_processing/filters.hpp"
#include "std_msgs/msg/float64.hpp"

namespace signal_processing
{

class SignalFilter : public rclcpp::Node
{
public:
  SignalFilter()
  : Node("signal_filter")
  {
    const double sample_rate_hz = declare_parameter<double>("sample_rate_hz", 1000.0);
    const double cutoff_hz = declare_parameter<double>("low_pass_cutoff_hz", 60.0);
    const std::int64_t median_window_size =
      declare_parameter<std::int64_t>("median_window_size", 5);

    if (median_window_size < 3 || median_window_size % 2 == 0) {
      throw std::invalid_argument("median_window_size must be an odd integer of at least 3");
    }

    low_pass_filter_ = std::make_unique<LowPassFilter>(cutoff_hz, sample_rate_hz);
    median_filter_ = std::make_unique<MedianFilter>(
      static_cast<std::size_t>(median_window_size));

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    low_pass_publisher_ =
      create_publisher<std_msgs::msg::Float64>("signal/low_pass", qos);
    median_publisher_ =
      create_publisher<std_msgs::msg::Float64>("signal/median", qos);
    noisy_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "signal/noisy", qos,
      std::bind(&SignalFilter::filter_sample, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Started: sample_rate=%.1f Hz, low_pass_cutoff=%.1f Hz, median_window=%ld",
      sample_rate_hz, cutoff_hz, static_cast<long>(median_window_size));
  }

private:
  void filter_sample(const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignored a non-finite input sample");
      return;
    }

    std_msgs::msg::Float64 low_pass_message;
    low_pass_message.data = low_pass_filter_->update(message->data);
    low_pass_publisher_->publish(low_pass_message);

    std_msgs::msg::Float64 median_message;
    median_message.data = median_filter_->update(message->data);
    median_publisher_->publish(median_message);
  }

  std::unique_ptr<LowPassFilter> low_pass_filter_;
  std::unique_ptr<MedianFilter> median_filter_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr low_pass_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr median_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr noisy_subscription_;
};

}  // namespace signal_processing

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<signal_processing::SignalFilter>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("signal_filter"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
