// SPDX-License-Identifier: Apache-2.0
#include "sgs/sim/plant.hpp"

#include "sgs/math.hpp"

#include <algorithm>
#include <cmath>

namespace sgs {
namespace sim {

namespace {

double apply_deadzone(double tau, double deadzone) {
  if (std::fabs(tau) < deadzone) {
    return 0.0;
  }
  return tau;
}

double coulomb(double w, double tau_c) {
  return tau_c * std::tanh(w / 0.05);
}

}  // namespace

void plant_step(double dt, const PlantParams& p, double tau_pitch_cmd, double tau_yaw_cmd,
                PlantState* st) {
  tau_pitch_cmd = apply_deadzone(tau_pitch_cmd, p.deadzone);
  tau_yaw_cmd = apply_deadzone(tau_yaw_cmd, p.deadzone);
  tau_pitch_cmd = std::clamp(tau_pitch_cmd, -p.tau_max, p.tau_max);
  tau_yaw_cmd = std::clamp(tau_yaw_cmd, -p.tau_max, p.tau_max);

  auto joint_accel = [&](double w_rel, double q_rel, double tau_cmd, double alpha_base) {
    const double spring = -p.k_spring * q_rel;
    const double damp = -p.b * w_rel;
    const double fric = -coulomb(w_rel, p.tau_c);
    return (tau_cmd + spring + damp + fric) / p.J + alpha_base;
  };

  const double alpha_theta =
      joint_accel(st->omega_theta, st->theta_rel, tau_pitch_cmd, st->base_omega_pitch);
  const double alpha_psi =
      joint_accel(st->omega_psi, st->psi_rel, tau_yaw_cmd, st->base_omega_yaw);

  st->omega_theta += alpha_theta * dt;
  st->omega_psi += alpha_psi * dt;
  st->theta_rel += st->omega_theta * dt;
  st->psi_rel += st->omega_psi * dt;

  if (st->theta_rel > p.theta_stop) {
    st->theta_rel = p.theta_stop;
    if (st->omega_theta > 0.0) {
      st->omega_theta = 0.0;
    }
  }
  if (st->theta_rel < -p.theta_stop) {
    st->theta_rel = -p.theta_stop;
    if (st->omega_theta < 0.0) {
      st->omega_theta = 0.0;
    }
  }
}

Quat world_from_camera(const PlantState& st) {
  const Quat q_base = quat_mul(quat_yaw(st.base_yaw), quat_pitch(st.base_pitch));
  const Quat q_roll = quat_from_axis_angle(1.0, 0.0, 0.0, st.base_roll);
  const Quat q_b = quat_mul(quat_mul(q_roll, q_base), quat_yaw(st.psi_rel));
  return quat_normalize(quat_mul(q_b, quat_pitch(st.theta_rel)));
}

}  // namespace sim
}  // namespace sgs
