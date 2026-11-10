// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

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
