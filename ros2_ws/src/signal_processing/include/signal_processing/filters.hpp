#ifndef SIGNAL_PROCESSING__FILTERS_HPP_
#define SIGNAL_PROCESSING__FILTERS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>
#include <vector>

namespace signal_processing
{

class LowPassFilter
{
public:
  LowPassFilter(double cutoff_hz, double sample_rate_hz)
  : alpha_(calculate_alpha(cutoff_hz, sample_rate_hz))
  {
  }

  double update(double input)
  {
    validate_input(input);

    if (!initialized_) {
      state_ = input;
      initialized_ = true;
    } else {
      state_ += alpha_ * (input - state_);
    }
    return state_;
  }

  void reset()
  {
    state_ = 0.0;
    initialized_ = false;
  }

  [[nodiscard]] double alpha() const
  {
    return alpha_;
  }

private:
  static double calculate_alpha(double cutoff_hz, double sample_rate_hz)
  {
    if (!std::isfinite(cutoff_hz) || !std::isfinite(sample_rate_hz) ||
      cutoff_hz <= 0.0 || sample_rate_hz <= 0.0)
    {
      throw std::invalid_argument("cutoff and sample rate must be finite and positive");
    }
    if (cutoff_hz >= sample_rate_hz / 2.0) {
      throw std::invalid_argument("cutoff must be lower than the Nyquist frequency");
    }

    constexpr double kPi = 3.14159265358979323846;
    return 1.0 - std::exp(-2.0 * kPi * cutoff_hz / sample_rate_hz);
  }

  static void validate_input(double input)
  {
    if (!std::isfinite(input)) {
      throw std::invalid_argument("filter input must be finite");
    }
  }

  double alpha_;
  double state_{0.0};
  bool initialized_{false};
};

class MedianFilter
{
public:
  explicit MedianFilter(std::size_t window_size)
  : window_size_(window_size)
  {
    if (window_size_ < 3U || window_size_ % 2U == 0U) {
      throw std::invalid_argument("median window size must be an odd integer of at least 3");
    }
  }

  double update(double input)
  {
    if (!std::isfinite(input)) {
      throw std::invalid_argument("filter input must be finite");
    }

    window_.push_back(input);
    if (window_.size() > window_size_) {
      window_.pop_front();
    }

    std::vector<double> sorted(window_.begin(), window_.end());
    const std::size_t middle = sorted.size() / 2U;
    std::nth_element(sorted.begin(), sorted.begin() + middle, sorted.end());
    const double upper = sorted[middle];

    if (sorted.size() % 2U == 1U) {
      return upper;
    }

    const double lower = *std::max_element(sorted.begin(), sorted.begin() + middle);
    return (lower + upper) / 2.0;
  }

  void reset()
  {
    window_.clear();
  }

  [[nodiscard]] std::size_t sample_count() const
  {
    return window_.size();
  }

private:
  std::size_t window_size_;
  std::deque<double> window_;
};

}  // namespace signal_processing

#endif  // SIGNAL_PROCESSING__FILTERS_HPP_
