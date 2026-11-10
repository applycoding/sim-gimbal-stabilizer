// SPDX-License-Identifier: Apache-2.0
#include "sgs/pid.hpp"

#include <cmath>

namespace sgs {

double pid_step(double dt, const PidGains& gains, double tau_d_s, double e, double meas,
                PidState* st, double umin, double umax, double deadzone, bool freeze_integ) {
  const double k_aw = gains.kp != 0.0 ? gains.ki / gains.kp : 0.0;
  double d_raw = 0.0;
  if (st->has_meas) {
    d_raw = (meas - st->meas_prev) / dt;
  }
  st->has_meas = true;
  st->meas_prev = meas;

  const double alpha = dt / (dt + tau_d_s);
  st->d_filt += alpha * (d_raw - st->d_filt);

  const double u_raw = gains.kp * e + st->integ - gains.kd * st->d_filt;
  double u_sat = u_raw;
  if (u_sat > umax) {
    u_sat = umax;
  }
  if (u_sat < umin) {
    u_sat = umin;
  }
  double u_applied = u_sat;
  if (std::fabs(u_sat) < deadzone) {
    u_applied = 0.0;
  }
  if (!freeze_integ) {
    st->integ += dt * (gains.ki * e + k_aw * (u_applied - u_raw));
  }
  return u_applied;
}

void pid_reset_derivative(PidState* st) {
  st->d_filt = 0.0;
  st->meas_prev = 0.0;
  st->has_meas = false;
}

}  // namespace sgs
