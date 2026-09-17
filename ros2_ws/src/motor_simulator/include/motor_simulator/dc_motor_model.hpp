#pragma once

#include <cmath>
#include <stdexcept>

namespace motor_simulator
{

struct MotorState
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
};

class DcMotorModel
{
public:
  DcMotorModel(double inertia, double damping, MotorState initial_state = {})
  : inertia_(inertia), damping_(damping), state_(initial_state)
  {
    if (!std::isfinite(inertia_) || inertia_ <= 0.0) {
      throw std::invalid_argument("inertia must be finite and greater than zero");
    }
    if (!std::isfinite(damping_) || damping_ < 0.0) {
      throw std::invalid_argument("damping must be finite and non-negative");
    }
  }

  void step(double control_torque, double load_torque, double dt)
  {
    if (!std::isfinite(control_torque) || !std::isfinite(load_torque)) {
      throw std::invalid_argument("torques must be finite");
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("time step must be finite and greater than zero");
    }

    // Fourth-order Runge-Kutta integration of:
    // theta_dot = omega
    // omega_dot = (control_torque - load_torque - damping * omega) / inertia
    const auto derivative = [this, control_torque, load_torque](const MotorState & x) {
        return MotorState{
          x.velocity_rad_s,
          (control_torque - load_torque - damping_ * x.velocity_rad_s) / inertia_};
      };

    const MotorState k1 = derivative(state_);
    const MotorState k2 = derivative(add_scaled(state_, k1, dt * 0.5));
    const MotorState k3 = derivative(add_scaled(state_, k2, dt * 0.5));
    const MotorState k4 = derivative(add_scaled(state_, k3, dt));

    state_.position_rad += dt *
      (k1.position_rad + 2.0 * k2.position_rad + 2.0 * k3.position_rad + k4.position_rad) /
      6.0;
    state_.velocity_rad_s += dt *
      (k1.velocity_rad_s + 2.0 * k2.velocity_rad_s + 2.0 * k3.velocity_rad_s +
      k4.velocity_rad_s) / 6.0;
  }

  const MotorState & state() const noexcept {return state_;}
  double inertia() const noexcept {return inertia_;}
  double damping() const noexcept {return damping_;}

private:
  static MotorState add_scaled(const MotorState & x, const MotorState & dx, double scale)
  {
    return MotorState{
      x.position_rad + dx.position_rad * scale,
      x.velocity_rad_s + dx.velocity_rad_s * scale};
  }

  double inertia_;
  double damping_;
  MotorState state_;
};

}  // namespace motor_simulator
