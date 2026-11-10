// SPDX-License-Identifier: Apache-2.0
#include "sgs/types.hpp"

#include "sgs/complementary.hpp"
#include "sgs/ekf.hpp"
#include "sgs/math.hpp"
#include "sgs/pid.hpp"

#include <cmath>
#include <cstring>

namespace sgs {

namespace {

void zero_output(StepOutput* out, Status st) {
  out->motor = {};
  out->motor.status = st;
  out->boresight_error_rad = {};
  out->omega_sp = {};
  out->pitch_rate_sp = 0.0;
  out->yaw_rate_sp = 0.0;
}

bool input_valid(const StepInput& in) {
  for (int i = 0; i < 3; ++i) {
    if (!is_finite(in.imu.gyro_rad_s[i])) {
      return false;
    }
  }
  if ((in.imu.flags & kFlagAccelValid) != 0) {
    for (int i = 0; i < 3; ++i) {
      if (!is_finite(in.imu.accel_m_s2[i])) {
        return false;
      }
    }
  }
  if (!is_finite(in.enc.pitch_rad) || !is_finite(in.enc.yaw_rad)) {
    return false;
  }
  const double n = vec3_norm(in.set.boresight_ned);
  if (n > 1e-9) {
    if (n < 0.9 || n > 1.1) {
      return false;
    }
    if (!vec3_finite(in.set.boresight_ned)) {
      return false;
    }
  }
  return true;
}

Vec3 resolve_boresight(const StepInput& in, ControllerState* state,
                       const Quat& q_world_from_camera) {
  const double n = vec3_norm(in.set.boresight_ned);
  if (n < 1e-9) {
    if (state->has_setpoint) {
      return state->last_boresight;
    }
    return camera_boresight_world(q_world_from_camera);
  }
  const Vec3 b = vec3_normalize(in.set.boresight_ned);
  state->last_boresight = b;
  state->has_setpoint = true;
  return b;
}

void clip_rate(Vec3* w, double limit) {
  if (w->x > limit) {
    w->x = limit;
  }
  if (w->x < -limit) {
    w->x = -limit;
  }
  if (w->y > limit) {
    w->y = limit;
  }
  if (w->y < -limit) {
    w->y = -limit;
  }
  if (w->z > limit) {
    w->z = limit;
  }
  if (w->z < -limit) {
    w->z = -limit;
  }
}

}  // namespace

void reset_state(ControllerState* state, const Quat& q_init) {
  std::memset(state, 0, sizeof(ControllerState));
  state->comp.q = quat_normalize(q_init);
  state->ekf = state->comp;
  ekf_init_state(&state->ekf);
  state->select = ControllerState::EstimatorSelect::Ekf;
  state->last_boresight = {1.0, 0.0, 0.0};
  state->has_setpoint = false;
  state->ready = true;
}

Status Controller::step(double dt, const Gains& gains, const Limits& limits,
                        const StepInput& in, ControllerState* state, StepOutput* out) const {
  if (state == nullptr || out == nullptr) {
    return Status::InvalidInput;
  }
  if (dt <= 0.0 || dt > 0.002 || !is_finite(dt)) {
    zero_output(out, Status::InvalidDt);
    return Status::InvalidDt;
  }
  if (!input_valid(in)) {
    zero_output(out, Status::InvalidInput);
    return Status::InvalidInput;
  }
  if (!state->ready) {
    zero_output(out, Status::NotReady);
    return Status::NotReady;
  }

  uint32_t est_flags = 0;
  EstimatorState* est = &state->ekf;
  if (state->select == ControllerState::EstimatorSelect::Complementary) {
    est = &state->comp;
    complementary_predict(dt, in.imu, gains.comp_kp_acc, gains.comp_kp_mag, est, &est_flags);
  } else {
    ekf_predict(dt, in.imu, gains, est);
    ekf_update(dt, in.imu, gains, limits, est, &est_flags);
  }

  const Quat& q = est->q;
  const Vec3 b_des = resolve_boresight(in, state, q);
  const Vec3 b_des_cam = vec3_normalize(quat_rotate(quat_conj(q), b_des));
  const Vec3 b_est_cam{1.0, 0.0, 0.0};
  const Vec3 b_err = boresight_error_camera(b_est_cam, b_des_cam);

  const double e_pitch = b_err.y;
  const double e_yaw = b_err.z;

  const double omega_pitch =
      pid_step(dt, gains.boresight, gains.tau_d_s, e_pitch, e_pitch, &state->att_pitch,
               -limits.rate_rad_s, limits.rate_rad_s, 0.0, false);
  const double omega_yaw =
      pid_step(dt, gains.boresight, gains.tau_d_s, e_yaw, e_yaw, &state->att_yaw,
               -limits.rate_rad_s, limits.rate_rad_s, 0.0, false);

  Vec3 omega_sp{0.0, omega_pitch, omega_yaw};
  clip_rate(&omega_sp, limits.rate_rad_s);

  const double theta = in.enc.pitch_rad;
  Vec3 omega_meas{in.imu.gyro_rad_s[0] - est->gyro_bias[0],
                  in.imu.gyro_rad_s[1] - est->gyro_bias[1],
                  in.imu.gyro_rad_s[2] - est->gyro_bias[2]};
  double theta_dot_meas = 0.0;
  double psi_dot_meas = 0.0;
  inertial_to_joint_rates(theta, omega_meas, &theta_dot_meas, &psi_dot_meas);

  const double theta_rel = theta;
  const bool near_warn = std::fabs(theta_rel) > limits.theta_warn_rad;
  const bool clear_hyst = std::fabs(theta_rel) < (limits.theta_warn_rad - 2.0 * kPi / 180.0);
  if (near_warn) {
    state->near_singularity_latched = true;
  } else if (clear_hyst) {
    state->near_singularity_latched = false;
  }

  const bool freeze_yaw = state->near_singularity_latched;

  const double rate_pitch_sp = omega_pitch;
  const double rate_yaw_sp = omega_yaw;
  const double m_pitch =
      pid_step(dt, gains.rate, gains.tau_d_s, rate_pitch_sp - theta_dot_meas, theta_dot_meas,
               &state->rate_pitch, -limits.tau_max_nm, limits.tau_max_nm, limits.deadzone_nm,
               false);
  const double m_yaw =
      pid_step(dt, gains.rate, gains.tau_d_s, rate_yaw_sp - psi_dot_meas, psi_dot_meas,
               &state->rate_yaw, -limits.tau_max_nm, limits.tau_max_nm, limits.deadzone_nm,
               freeze_yaw);

  const Vec3 m_cam{0.0, m_pitch, m_yaw};
  double tau_pitch = 0.0;
  double tau_yaw = 0.0;
  camera_effort_to_torques(theta, m_cam, &tau_pitch, &tau_yaw);

  uint32_t flags = est_flags;
  if (std::fabs(tau_pitch) >= limits.tau_max_nm - 1e-9) {
    flags |= kFlagSatPitch;
  }
  if (std::fabs(tau_yaw) >= limits.tau_max_nm - 1e-9) {
    flags |= kFlagSatYaw;
  }
  if (state->near_singularity_latched) {
    flags |= kFlagNearSingularity;
  }

  out->motor.tau_pitch = tau_pitch;
  out->motor.tau_yaw = tau_yaw;
  out->motor.status = Status::Ok;
  out->motor.flags = flags;
  out->boresight_error_rad = b_err;
  out->omega_sp = omega_sp;
  out->pitch_rate_sp = rate_pitch_sp;
  out->yaw_rate_sp = rate_yaw_sp;
  return Status::Ok;
}

}  // namespace sgs
