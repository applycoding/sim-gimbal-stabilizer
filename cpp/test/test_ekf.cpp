// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

#include "sgs/complementary.hpp"
#include "sgs/ekf.hpp"
#include "sgs/math.hpp"

TEST(Ekf, KalmanHandExample) {
  double x = 0.0;
  double P = 1.0;
  const double K = sgs::kalman_update_scalar(&x, &P, 1.0, 1.0, 1.0);
  EXPECT_NEAR(K, 0.5, 1e-12);
  EXPECT_NEAR(x, 0.5, 1e-12);
  EXPECT_NEAR(P, 0.5, 1e-12);
}

TEST(Ekf, YawBiasConvergence) {
  sgs::EstimatorState st{};
  sgs::ekf_init_state(&st);
  sgs::Gains g = sgs::default_gains();
  sgs::Limits L = sgs::default_limits();
  sgs::ImuSample imu{};
  imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  imu.gyro_rad_s[2] = 0.01;
  imu.accel_m_s2[2] = -sgs::kG;
  imu.mag_uT[0] = 25;
  imu.mag_uT[2] = 40;
  uint32_t flags = 0;
  for (int i = 0; i < 8000; ++i) {
    sgs::ekf_predict(0.001, imu, g, &st);
    sgs::ekf_update(0.001, imu, g, L, &st, &flags);
  }
  EXPECT_LT(std::fabs(st.gyro_bias[2] - 0.01), 0.003);
}

TEST(Ekf, MagYawCorrection) {
  sgs::EstimatorState st{};
  sgs::ekf_init_state(&st);
  const double yaw0 = 22.0 * sgs::kPi / 180.0;
  st.q = sgs::quat_yaw(yaw0);
  sgs::Gains g = sgs::default_gains();
  sgs::Limits L = sgs::default_limits();
  const sgs::Quat q_true{1.0, 0.0, 0.0, 0.0};
  const sgs::Vec3 m_ref{25.0, 0.0, 40.0};
  const sgs::Vec3 m_body = sgs::quat_rotate(sgs::quat_conj(q_true), m_ref);
  // Specific force f = R^T (-g_ned); a level camera reports [0, 0, -g].
  const sgs::Vec3 f_body = sgs::quat_rotate(sgs::quat_conj(q_true), {0.0, 0.0, -sgs::kG});
  sgs::ImuSample imu{};
  imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  imu.mag_uT[0] = m_body.x;
  imu.mag_uT[1] = m_body.y;
  imu.mag_uT[2] = m_body.z;
  imu.accel_m_s2[0] = f_body.x;
  imu.accel_m_s2[1] = f_body.y;
  imu.accel_m_s2[2] = f_body.z;
  const double err0 = std::fabs(sgs::mag_heading_error_rad(st.q, imu));
  EXPECT_GT(err0, 0.15);
  uint32_t flags = 0;
  for (int i = 0; i < 8000; ++i) {
    sgs::ekf_predict(0.001, imu, g, &st);
    sgs::ekf_update(0.001, imu, g, L, &st, &flags);
  }
  const double err1 = std::fabs(sgs::mag_heading_error_rad(st.q, imu));
  EXPECT_LT(err1, err0);
  EXPECT_LT(err1, 8.0 * sgs::kPi / 180.0);
  EXPECT_EQ(flags & sgs::kFlagMagWeak, 0u);
}

TEST(Ekf, StaticBoresightMagOn) {
  sgs::EstimatorState st{};
  sgs::ekf_init_state(&st);
  sgs::Gains g = sgs::default_gains();
  sgs::Limits L = sgs::default_limits();
  const sgs::Quat q_true{1.0, 0.0, 0.0, 0.0};
  const sgs::Vec3 m_ref{25.0, 0.0, 40.0};
  const sgs::Vec3 m_body = sgs::quat_rotate(sgs::quat_conj(q_true), m_ref);
  // Specific force f = R^T (-g_ned); a level camera reports [0, 0, -g].
  const sgs::Vec3 f_body = sgs::quat_rotate(sgs::quat_conj(q_true), {0.0, 0.0, -sgs::kG});
  sgs::ImuSample imu{};
  imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  imu.gyro_rad_s[2] = 0.01;
  imu.mag_uT[0] = m_body.x;
  imu.mag_uT[1] = m_body.y;
  imu.mag_uT[2] = m_body.z;
  imu.accel_m_s2[0] = f_body.x;
  imu.accel_m_s2[1] = f_body.y;
  imu.accel_m_s2[2] = f_body.z;
  uint32_t flags = 0;
  for (int i = 0; i < 8000; ++i) {
    sgs::ekf_predict(0.001, imu, g, &st);
    sgs::ekf_update(0.001, imu, g, L, &st, &flags);
  }
  EXPECT_LT(std::fabs(st.gyro_bias[2] - 0.01), 0.003);
  const sgs::Vec3 b_est = sgs::camera_boresight_world(st.q);
  const sgs::Vec3 b_des{1.0, 0.0, 0.0};
  const sgs::Vec3 err = sgs::boresight_error_camera(b_est, b_des);
  const double bore_err = std::sqrt(err.x * err.x + err.y * err.y + err.z * err.z);
  EXPECT_LT(bore_err, 1.0 * sgs::kPi / 180.0);
}

TEST(Ekf, MagOffNoYawObserve) {
  sgs::EstimatorState st{};
  sgs::ekf_init_state(&st);
  sgs::Gains g = sgs::default_gains();
  sgs::Limits L = sgs::default_limits();
  sgs::ImuSample imu{};
  imu.flags = sgs::kFlagAccelValid;
  imu.gyro_rad_s[2] = 0.01;
  imu.accel_m_s2[2] = -sgs::kG;
  const double bias0 = st.gyro_bias[2];
  uint32_t flags = 0;
  for (int i = 0; i < 8000; ++i) {
    sgs::ekf_predict(0.001, imu, g, &st);
    sgs::ekf_update(0.001, imu, g, L, &st, &flags);
  }
  EXPECT_NEAR(st.gyro_bias[2], bias0, 1e-5);
}
