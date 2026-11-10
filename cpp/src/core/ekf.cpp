// SPDX-License-Identifier: Apache-2.0
#include "sgs/ekf.hpp"

#include "sgs/complementary.hpp"
#include "sgs/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace sgs {

namespace {

constexpr int kN = 6;

void mat6_identity(double P[36]) {
  std::memset(P, 0, sizeof(double) * 36);
  for (int i = 0; i < kN; ++i) {
    P[i * kN + i] = 1.0;
  }
}

void inject_error(EstimatorState* st, const double dx[6]) {
  const Vec3 dth{dx[0], dx[1], dx[2]};
  const Quat dq = quat_exp(dth, 1.0);
  st->q = quat_normalize(quat_mul(st->q, dq));
  quat_shortest_path(&st->q);
  for (int i = 0; i < 3; ++i) {
    st->gyro_bias[i] += dx[3 + i];
  }
}

void joseph_update(double P[36], const double K[6], double H, double R, double innov) {
  (void)innov;
  double PHt[6]{};
  for (int i = 0; i < kN; ++i) {
    PHt[i] = 0.0;
    for (int j = 0; j < kN; ++j) {
      PHt[i] += P[i * kN + j] * H;
    }
  }
  const double S = H * PHt[0] + R;
  if (S < 1e-18) {
    return;
  }
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      P[i * kN + j] -= K[i] * PHt[j];
    }
  }
  for (int i = 0; i < kN; ++i) {
    P[i * kN + i] = std::max(P[i * kN + i], 1e-12);
  }
}

}  // namespace

double kalman_update_scalar(double* x, double* P, double z, double R, double H) {
  const double S = (*P) * H * H + R;
  const double K = (*P) * H / S;
  const double innov = z - H * (*x);
  *x += K * innov;
  *P = (1.0 - K * H) * (*P);
  return K;
}

void ekf_init_state(EstimatorState* st) {
  st->q = {1.0, 0.0, 0.0, 0.0};
  std::memset(st->gyro_bias, 0, sizeof(st->gyro_bias));
  mat6_identity(st->P);
  for (int i = 0; i < 3; ++i) {
    st->P[i * kN + i] = 0.02;
    st->P[(3 + i) * kN + (3 + i)] = 1e-5;
  }
}

void ekf_predict(double dt, const ImuSample& imu, const Gains& gains, EstimatorState* st) {
  const Vec3 w{imu.gyro_rad_s[0] - st->gyro_bias[0], imu.gyro_rad_s[1] - st->gyro_bias[1],
               imu.gyro_rad_s[2] - st->gyro_bias[2]};
  const Quat dq = quat_exp(w, dt);
  st->q = quat_normalize(quat_mul(st->q, dq));
  quat_shortest_path(&st->q);

  const double qg = gains.ekf_sigma_gyro * gains.ekf_sigma_gyro * dt;
  const double qb = gains.ekf_sigma_bias_rw * gains.ekf_sigma_bias_rw * dt;
  for (int i = 0; i < 3; ++i) {
    st->P[i * kN + i] += qg;
    st->P[(3 + i) * kN + (3 + i)] += qb;
  }

}

void ekf_update(double dt, const ImuSample& imu, const Gains& gains, const Limits& limits,
                EstimatorState* st, uint32_t* flags) {
  (void)dt;
  double dx[6]{};

  if ((imu.flags & kFlagAccelValid) != 0) {
    const Vec3 f{imu.accel_m_s2[0], imu.accel_m_s2[1], imu.accel_m_s2[2]};
    const double fn = vec3_norm(f);
    const double gate = std::fabs(fn - kG) / kG;
    if (gate > limits.accel_gate_frac) {
      *flags |= kFlagAccelGated;
    } else {
      const Quat qc = quat_conj(st->q);
      const Vec3 f_pred = quat_rotate(qc, Vec3{0.0, 0.0, -kG});
      const Vec3 f_hat = vec3_normalize(f);
      const Vec3 f_pred_hat = vec3_normalize(f_pred);
      const Vec3 e = vec3_cross(f_hat, f_pred_hat);
      const double ecomp[3] = {e.x, e.y, e.z};
      const double innov = vec3_norm(e);
      if (innov < 25.0 * kPi / 180.0) {
        const double R = gains.ekf_sigma_acc * gains.ekf_sigma_acc;
        for (int axis = 0; axis < 3; ++axis) {
          double x = 0.0;
          double P = st->P[axis * kN + axis];
          kalman_update_scalar(&x, &P, ecomp[axis], R, 1.0);
          st->P[axis * kN + axis] = P;
          dx[axis] += x;
        }
      } else {
        *flags |= kFlagAccelGated;
      }
    }
  }

  const bool mag_valid = (imu.flags & kFlagMagValid) != 0;
  if (!mag_valid) {
    *flags |= kFlagMagMissing;
  } else {
    const Vec3 m{imu.mag_uT[0], imu.mag_uT[1], imu.mag_uT[2]};
    const double mn = vec3_norm(m);
    const double refn =
        std::sqrt(complementary::kMagRefNed[0] * complementary::kMagRefNed[0] +
                  complementary::kMagRefNed[1] * complementary::kMagRefNed[1] +
                  complementary::kMagRefNed[2] * complementary::kMagRefNed[2]);
    if (mn < 1e-6) {
      *flags |= kFlagMagWeak;
    } else if (std::fabs(mn - refn) / refn > limits.mag_gate_frac) {
      *flags |= kFlagMagWeak;
    } else {
      const double horiz_body = std::sqrt(m.x * m.x + m.y * m.y);
      if (horiz_body < 5.0) {
        *flags |= kFlagMagWeak;
      } else {
        const double yaw_innov = mag_heading_error_rad(st->q, imu);
        if (std::fabs(yaw_innov) > 30.0 * kPi / 180.0) {
          *flags |= kFlagMagWeak;
        } else {
          const double R = gains.ekf_sigma_mag * gains.ekf_sigma_mag;
          double x = 0.0;
          double P = st->P[2 * kN + 2];
          kalman_update_scalar(&x, &P, yaw_innov, R, 1.0);
          dx[2] -= x;
          st->P[2 * kN + 2] = P;
        }
      }
    }
  }

  if (dx[0] != 0.0 || dx[1] != 0.0 || dx[2] != 0.0) {
    inject_error(st, dx);
    std::memset(dx, 0, sizeof(dx));
  }

  // v1 deviation: yaw/tilt bias is a stationary-gate IIR on the gyro, not a
  // Kalman update from the magnetometer innovation. See DEVIATIONS.md.
  if ((imu.flags & kFlagMagValid) != 0 && (imu.flags & kFlagAccelValid) != 0) {
  const double wx = imu.gyro_rad_s[0];
  const double wy = imu.gyro_rad_s[1];
  const double wz = imu.gyro_rad_s[2];
  const Vec3 a{imu.accel_m_s2[0], imu.accel_m_s2[1], imu.accel_m_s2[2]};
  const double gerr = std::fabs(vec3_norm(a) - kG) / kG;
  const bool nearly_static =
      std::fabs(wx) < 0.012 && std::fabs(wy) < 0.012 && std::fabs(wz) < 0.012 && gerr < 0.05;
  if (nearly_static) {
    if (st->static_gyro_count < 4000) {
      ++st->static_gyro_count;
    }
  } else {
    st->static_gyro_count = 0;
  }
  if (st->static_gyro_count >= 1500) {
    constexpr double alpha = 0.004;
    for (int i = 0; i < 3; ++i) {
      st->gyro_bias[i] += alpha * (imu.gyro_rad_s[i] - st->gyro_bias[i]);
    }
  }
  }
}

}  // namespace sgs
