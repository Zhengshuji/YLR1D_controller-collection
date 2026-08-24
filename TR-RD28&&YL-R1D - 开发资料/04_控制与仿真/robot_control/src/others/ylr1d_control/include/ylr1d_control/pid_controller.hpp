/*****************************************************************************
 * ylr1d_pid_controller.hpp
 *
 * Per-joint PID position controller.
 *
 * Control law (for each joint i):
 *   τᵢ = kpᵢ · (q_desᵢ − q_actᵢ) + kiᵢ · ∫(q_desᵢ − q_actᵢ)dt − kdᵢ · q̇_actᵢ
 *
 * The derivative term uses negative velocity feedback (rather than the
 * derivative of error) for numerical stability — equivalent to "PID with
 * derivative on measurement".
 *
 * Anti-windup: integral accumulation is disabled when the output torque
 * saturates (clamped at ±max_effort).
 *****************************************************************************/

#ifndef YLR1D_CONTROL__PID_CONTROLLER_HPP_
#define YLR1D_CONTROL__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ylr1d_control
{

struct PidGains
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double max_effort = 100.0;
  double i_max = 0.0;  // integral term clamp (0 = no clamp)
};

struct PidState
{
  double integral = 0.0;
  double prev_error = 0.0;
  bool first_run = true;
};

class PidController
{
public:
  /// Configure gains for one joint.
  void set_gains(const PidGains &gains) { gains_ = gains; }
  const PidGains &gains() const { return gains_; }

  /// Reset integral term and prev_error.
  void reset()
  {
    state_.integral = 0.0;
    state_.prev_error = 0.0;
    state_.first_run = true;
  }

  /// Compute one PID step.
  /// @param q_desired  Target position (rad)
  /// @param q_actual   Current position (rad)
  /// @param qd_actual  Current velocity (rad/s)
  /// @param dt         Time step (s)
  /// @return Effort command (N·m)
  double compute(double q_desired, double q_actual, double qd_actual, double dt)
  {
    double error = q_desired - q_actual;

    if (state_.first_run) {
      state_.first_run = false;
      state_.integral = 0.0;
      state_.prev_error = error;
    }

    // Integral with trapezoidal integration
    state_.integral += (error + state_.prev_error) * 0.5 * dt;

    // Clamp integral term
    if (gains_.i_max > 0.0)
      state_.integral = std::clamp(state_.integral, -gains_.i_max, gains_.i_max);

    // PID output (derivative on measurement: −kd·q̇ for stability)
    double effort = gains_.kp * error
                  + gains_.ki * state_.integral
                  - gains_.kd * qd_actual;

    // Anti-windup: if saturated, undo integral accumulation
    if (std::abs(effort) > gains_.max_effort) {
      state_.integral -= (error + state_.prev_error) * 0.5 * dt;
      effort = std::clamp(effort, -gains_.max_effort, gains_.max_effort);
    }

    state_.prev_error = error;
    return effort;
  }

private:
  PidGains gains_;
  PidState state_;
};

}  // namespace ylr1d_control

#endif  // YLR1D_CONTROL__PID_CONTROLLER_HPP_
