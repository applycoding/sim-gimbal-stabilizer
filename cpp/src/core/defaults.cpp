// SPDX-License-Identifier: Apache-2.0
#include "sgs/types.hpp"

#include "sgs/math.hpp"

namespace sgs {

Gains default_gains() {
  Gains g{};
  g.rate.kp = 0.05;
  g.rate.ki = 0.40;
  g.rate.kd = 2e-4;
  g.boresight.kp = 10.0;
  g.boresight.ki = 2.0;
  g.boresight.kd = 0.10;
  g.tau_d_s = 0.005;
  g.ekf_sigma_gyro = 0.01;
  g.ekf_sigma_bias_rw = 1e-4;
  g.ekf_sigma_acc = 0.3;
  g.ekf_sigma_mag = 1.0;
  g.comp_kp_acc = 2.0;
  g.comp_kp_mag = 0.5;
  return g;
}

Limits default_limits() {
  Limits L{};
  L.rate_rad_s = 2.0;
  L.tau_max_nm = 0.15;
  L.deadzone_nm = 0.001;
  L.theta_warn_rad = 75.0 * kPi / 180.0;
  L.accel_gate_frac = 0.15;
  L.mag_gate_frac = 0.30;
  return L;
}

}  // namespace sgs
