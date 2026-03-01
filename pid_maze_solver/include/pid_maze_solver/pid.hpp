#ifndef MAZE_SOLVER__PID_HPP_
#define MAZE_SOLVER__PID_HPP_

#include <algorithm>

namespace maze_solver {

class PID {
public:
  PID() = default;
  PID(double p, double i, double d) : kp_(p), ki_(i), kd_(d) {}

  void set_gain(double p, double i, double d) {
    kp_ = p; ki_ = i; kd_ = d;
  }

  void set_limit(double out_limit, double integ_limit) {
    out_limit_   = out_limit;
    integ_limit_ = integ_limit;
  }

  void reset() {
    integral_ = 0.0;
    prev_err_ = 0.0;
  }

  double compute(double err, double dt) {
    if (dt <= 0.0) return 0.0;
    dt = std::clamp(dt, 0.001, 0.1);  // prevent instability

    double p_term = kp_ * err;

    integral_ += err * dt;
    integral_  = std::clamp(integral_, -integ_limit_, integ_limit_);
    double i_term = ki_ * integral_;

    double d_term = kd_ * (err - prev_err_) / dt;
    prev_err_ = err;

    double output = p_term + i_term + d_term;
    return std::clamp(output, -out_limit_, out_limit_);
  }

private:
  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double integral_{0.0};
  double prev_err_{0.0};
  double integ_limit_{1.0};  // safe default
  double out_limit_{1.0};    // safe default
};

} // namespace maze_solver

#endif // MAZE_SOLVER__PID_HPP_