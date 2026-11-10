// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "sgs/pid.hpp"

TEST(Pid, Integrator) {
  sgs::PidGains g{0, 1.0, 0};
  sgs::PidState st{};
  sgs::pid_step(0.001, g, 0.005, 1.0, 0.0, &st, -10, 10, 0, false);
  const double u = sgs::pid_step(0.001, g, 0.005, 1.0, 0.0, &st, -10, 10, 0, false);
  EXPECT_NEAR(u, 0.001, 1e-6);
  EXPECT_NEAR(st.integ, 0.002, 1e-9);
}

TEST(Pid, Deadzone) {
  sgs::PidGains g{1.0, 0, 0};
  sgs::PidState st{};
  const double u = sgs::pid_step(0.001, g, 0.005, 0.0005, 0.0, &st, -10, 10, 0.001, false);
  EXPECT_DOUBLE_EQ(u, 0.0);
}

TEST(Pid, SaturationUnwind) {
  sgs::PidGains g{10.0, 5.0, 0};
  sgs::PidState st{};
  double u = sgs::pid_step(0.001, g, 0.005, 1.0, 0.0, &st, -1.0, 1.0, 0, false);
  EXPECT_NEAR(u, 1.0, 1e-9);
  u = sgs::pid_step(0.001, g, 0.005, 1.0, 0.0, &st, -1.0, 1.0, 0, false);
  EXPECT_LT(st.integ, 5.0);
}
