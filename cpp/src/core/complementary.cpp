// SPDX-License-Identifier: Apache-2.0
#include "sgs/complementary.hpp"

#include "sgs/math.hpp"

#include <cmath>

namespace sgs {

double mag_heading_error_rad(const Quat& q, const ImuSample& imu) {
  const Vec3 m_body{imu.mag_uT[0], imu.mag_uT[1], imu.mag_uT[2]};
  const double mn = vec3_norm(m_body);
  if (mn < 1e-6) {
    return 0.0;
  }
  const Vec3 m_hat = vec3_scale(m_body, 1.0 / mn);
  const Vec3 m_ned = quat_rotate(q, m_hat);
  const Vec3 ref{complementary::kMagRefNed[0], complementary::kMagRefNed[1],
               complementary::kMagRefNed[2]};
  const double rn = vec3_norm(ref);
  if (rn < 1e-6) {
    return 0.0;
  }
  const double ref_h = std::atan2(ref.y, ref.x);
  const double est_h = std::atan2(m_ned.y, m_ned.x);
  double err = est_h - ref_h;
  while (err > kPi) {
    err -= 2.0 * kPi;
  }
  while (err < -kPi) {
    err += 2.0 * kPi;
  }
  return err;
}

void complementary_predict(double dt, const ImuSample& imu, double comp_kp_acc,
                           double comp_kp_mag, EstimatorState* st, uint32_t* flags) {
  Vec3 omega{imu.gyro_rad_s[0], imu.gyro_rad_s[1], imu.gyro_rad_s[2]};

  // Mahony-style: unit specific-force error in the camera frame.
  // f_pred = R^T (0 - g_ned) so a level camera measures [0, 0, -g].
  if ((imu.flags & kFlagAccelValid) != 0) {
    const Vec3 f{imu.accel_m_s2[0], imu.accel_m_s2[1], imu.accel_m_s2[2]};
    const double fn = vec3_norm(f);
    if (fn > 1e-9) {
      const Vec3 f_hat = vec3_scale(f, 1.0 / fn);
      const Vec3 f_pred = quat_rotate(quat_conj(st->q), Vec3{0.0, 0.0, -kG});
      const double pn = vec3_norm(f_pred);
      if (pn > 1e-9) {
        const Vec3 f_pred_hat = vec3_scale(f_pred, 1.0 / pn);
        const Vec3 corr = vec3_cross(f_hat, f_pred_hat);
        omega = vec3_add(omega, vec3_scale(corr, comp_kp_acc));
      }
    }
  }

  const bool mag_valid = (imu.flags & kFlagMagValid) != 0;
  if (!mag_valid) {
    *flags |= kFlagMagMissing;
  } else {
    const double yaw_err = mag_heading_error_rad(st->q, imu);
    omega.z += comp_kp_mag * yaw_err;
  }

  const Quat dq = quat_exp(omega, dt);
  st->q = quat_normalize(quat_mul(st->q, dq));
  quat_shortest_path(&st->q);
}

}  // namespace sgs
