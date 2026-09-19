#pragma once

#include <cmath>
#include <stdexcept>

namespace pid_motor_control {

struct PidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

struct PidLimits {
    double output_min{-1.0};
    double output_max{1.0};
    double integral_min{-1.0};
    double integral_max{1.0};
};

class PidController {
public:
    PidController(PidGains gains, PidLimits limits)
        : gains_(gains)
        , limits_(limits) {
        validate_configuration();
    }

    double update(double setpoint, double measurement, double dt) {
        if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
            throw std::invalid_argument("PID inputs must be finite");
        }
        return update_error(setpoint - measurement, dt);
    }

    double update_error(double error, double dt) {
        if (!std::isfinite(error)) {
            throw std::invalid_argument("PID error must be finite");
        }
        if (!std::isfinite(dt) || dt <= 0.0) {
            throw std::invalid_argument("PID dt must be finite and greater than zero");
        }

        const double derivative = initialized_ ? (error - previous_error_) / dt : 0.0;
        integral_ += error * dt;
        previous_error_ = error;
        initialized_ = true;
        return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    }

    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
        initialized_ = false;
    }

    double integral() const noexcept { return integral_; }
    bool initialized() const noexcept { return initialized_; }

private:
    void validate_configuration() const {
        if (!std::isfinite(gains_.kp) || !std::isfinite(gains_.ki) || !std::isfinite(gains_.kd)
            || gains_.kp < 0.0 || gains_.ki < 0.0 || gains_.kd < 0.0) {
            throw std::invalid_argument("PID gains must be finite and non-negative");
        }
        if (!std::isfinite(limits_.output_min) || !std::isfinite(limits_.output_max)
            || limits_.output_min >= limits_.output_max) {
            throw std::invalid_argument("PID output limits are invalid");
        }
        if (!std::isfinite(limits_.integral_min) || !std::isfinite(limits_.integral_max)
            || limits_.integral_min >= limits_.integral_max) {
            throw std::invalid_argument("PID integral limits are invalid");
        }
    }

    PidGains gains_;
    PidLimits limits_;
    double integral_{0.0};
    double previous_error_{0.0};
    bool initialized_{false};
};

}
