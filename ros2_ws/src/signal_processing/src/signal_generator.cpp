#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace signal_processing
{

class SignalGenerator : public rclcpp::Node
{
public:
  SignalGenerator()
  : Node("signal_generator")
  {
    amplitude_ = declare_parameter<double>("amplitude", 1.0);
    signal_frequency_hz_ = declare_parameter<double>("signal_frequency_hz", 20.0);
    sample_rate_hz_ = declare_parameter<double>("sample_rate_hz", 1000.0);
    noise_ratio_ = declare_parameter<double>("noise_ratio", 0.01);
    const std::int64_t random_seed = declare_parameter<std::int64_t>("random_seed", 42);

    validate_parameters(random_seed);

    noise_standard_deviation_ = amplitude_ * noise_ratio_;
    if (!std::isfinite(noise_standard_deviation_)) {
      throw std::invalid_argument("amplitude multiplied by noise_ratio must be finite");
    }
    random_engine_.seed(static_cast<std::mt19937::result_type>(random_seed));
    if (noise_standard_deviation_ > 0.0) {
      noise_distribution_ =
        std::normal_distribution<double>(0.0, noise_standard_deviation_);
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    clean_publisher_ =
      create_publisher<std_msgs::msg::Float64>("signal/clean", qos);
    noisy_publisher_ =
      create_publisher<std_msgs::msg::Float64>("signal/noisy", qos);

    start_time_ = std::chrono::steady_clock::now();
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / sample_rate_hz_));
    timer_ = create_wall_timer(timer_period, std::bind(&SignalGenerator::publish_sample, this));

    RCLCPP_INFO(
      get_logger(),
      "Started: amplitude=%.3f V, frequency=%.1f Hz, sample_rate=%.1f Hz, noise_sigma=%.4f V",
      amplitude_, signal_frequency_hz_, sample_rate_hz_, noise_standard_deviation_);
  }

private:
  void validate_parameters(std::int64_t random_seed) const
  {
    if (!std::isfinite(amplitude_) || amplitude_ <= 0.0) {
      throw std::invalid_argument("amplitude must be finite and greater than zero");
    }
    if (!std::isfinite(signal_frequency_hz_) || signal_frequency_hz_ <= 0.0) {
      throw std::invalid_argument("signal_frequency_hz must be finite and greater than zero");
    }
    if (!std::isfinite(sample_rate_hz_) || sample_rate_hz_ <= 2.0 * signal_frequency_hz_) {
      throw std::invalid_argument("sample_rate_hz must be above twice the signal frequency");
    }
    if (!std::isfinite(noise_ratio_) || noise_ratio_ < 0.0) {
      throw std::invalid_argument("noise_ratio must be finite and non-negative");
    }
    if (random_seed < 0 ||
      static_cast<std::uint64_t>(random_seed) >
      static_cast<std::uint64_t>(std::numeric_limits<std::mt19937::result_type>::max()))
    {
      throw std::invalid_argument("random_seed must fit in an unsigned 32-bit integer");
    }
    if (sample_rate_hz_ <= 500.0) {
      RCLCPP_WARN(
        get_logger(),
        "sample_rate_hz is %.1f Hz; the assignment recommends a value above 500 Hz",
        sample_rate_hz_);
    }
  }

  void publish_sample()
  {
    constexpr double kPi = 3.14159265358979323846;
    const double elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    const double clean_value = amplitude_ * std::sin(
      2.0 * kPi * signal_frequency_hz_ * elapsed_seconds);

    std_msgs::msg::Float64 clean_message;
    clean_message.data = clean_value;
    clean_publisher_->publish(clean_message);

    std_msgs::msg::Float64 noisy_message;
    noisy_message.data = clean_value;
    if (noise_standard_deviation_ > 0.0) {
      noisy_message.data += noise_distribution_(random_engine_);
    }
    noisy_publisher_->publish(noisy_message);
  }

  double amplitude_{1.0};
  double signal_frequency_hz_{20.0};
  double sample_rate_hz_{1000.0};
  double noise_ratio_{0.01};
  double noise_standard_deviation_{0.01};

  std::mt19937 random_engine_;
  std::normal_distribution<double> noise_distribution_;
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr clean_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr noisy_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace signal_processing

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<signal_processing::SignalGenerator>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("signal_generator"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
