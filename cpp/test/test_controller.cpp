// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

#include "sgs/types.hpp"

TEST(Controller, InvalidDt) {
  sgs::Controller c;
  sgs::ControllerState st{};
  sgs::reset_state(&st, {1, 0, 0, 0});
  sgs::StepInput in{};
  sgs::StepOutput out{};
  in.imu.flags = sgs::kFlagAccelValid;
  EXPECT_EQ(c.step(0.05, sgs::default_gains(), sgs::default_limits(), in, &st, &out),
            sgs::Status::InvalidDt);
  EXPECT_EQ(out.motor.tau_pitch, 0.0);
}

TEST(Controller, NearSingularity85) {
  sgs::Controller c;
  sgs::ControllerState st{};
  sgs::reset_state(&st, {1, 0, 0, 0});
  sgs::StepInput in{};
  in.imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  in.imu.accel_m_s2[2] = -9.80665;
  in.imu.mag_uT[0] = 25;
  in.imu.mag_uT[2] = 40;
  in.enc.pitch_rad = 85.0 * 3.14159265358979323846 / 180.0;
  in.set.boresight_ned = {1, 0, 0};
  sgs::StepOutput out{};
  c.step(0.001, sgs::default_gains(), sgs::default_limits(), in, &st, &out);
  EXPECT_TRUE(out.motor.flags & sgs::kFlagNearSingularity);
  EXPECT_TRUE(std::isfinite(out.motor.tau_yaw));
}
