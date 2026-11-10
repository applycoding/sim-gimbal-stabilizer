// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

#include "sgs/complementary.hpp"
#include "sgs/math.hpp"

TEST(Complementary, StaticTilt20Deg) {
  sgs::EstimatorState st{};
  st.q = {1, 0, 0, 0};
  const double tilt = 20.0 * sgs::kPi / 180.0;
  sgs::ImuSample imu{};
  imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  const sgs::Quat q_true = sgs::quat_pitch(tilt);
  // Specific force f = R^T (-g_ned)
  const sgs::Vec3 f = sgs::quat_rotate(sgs::quat_conj(q_true), {0, 0, -sgs::kG});
  imu.accel_m_s2[0] = f.x;
  imu.accel_m_s2[1] = f.y;
  imu.accel_m_s2[2] = f.z;
  imu.mag_uT[0] = 25;
  imu.mag_uT[2] = 40;
  uint32_t flags = 0;
  for (int i = 0; i < 3000; ++i) {
    sgs::complementary_predict(0.001, imu, 2.0, 0.5, &st, &flags);
  }
  const sgs::Vec3 b = sgs::camera_boresight_world(st.q);
  EXPECT_NEAR(std::asin(-b.z), tilt, 1.0 * sgs::kPi / 180.0);
}

TEST(Complementary, BiasedGyroEquilibrium) {
  sgs::EstimatorState st{};
  sgs::ImuSample imu{};
  imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  imu.gyro_rad_s[1] = 0.01;
  imu.accel_m_s2[2] = -sgs::kG;
  imu.mag_uT[0] = 25;
  imu.mag_uT[2] = 40;
  uint32_t flags = 0;
  for (int i = 0; i < 5000; ++i) {
    sgs::complementary_predict(0.001, imu, 2.0, 0.5, &st, &flags);
  }
  const double pred = 0.01 / 2.0;
  const sgs::Vec3 b = sgs::camera_boresight_world(st.q);
  // Positive pitch is up in FRD: +x toward -z, so pitch = asin(-b.z).
  EXPECT_NEAR(std::asin(-b.z), pred, 0.15 * sgs::kPi / 180.0);
}
