// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

#include "sgs/math.hpp"

TEST(Math, Pitch90Basis) {
  const sgs::Quat q = sgs::quat_pitch(sgs::kPi / 2.0);
  const sgs::Vec3 x{1, 0, 0};
  const sgs::Vec3 v = sgs::quat_rotate(q, x);
  EXPECT_NEAR(v.x, 0.0, 1e-9);
  EXPECT_NEAR(v.y, 0.0, 1e-9);
  EXPECT_NEAR(v.z, -1.0, 1e-9);
}

TEST(Math, LsMapAt0And30) {
  const sgs::Vec3 w{0.1, 0.2, 0.3};
  double td = 0, pd = 0;
  sgs::inertial_to_joint_rates(0.0, w, &td, &pd);
  EXPECT_NEAR(td, 0.2, 1e-12);
  EXPECT_NEAR(pd, 0.3, 1e-12);
  sgs::inertial_to_joint_rates(30.0 * sgs::kPi / 180.0, w, &td, &pd);
  EXPECT_NEAR(td, 0.2, 1e-12);
  EXPECT_NEAR(pd, -std::sin(30.0 * sgs::kPi / 180.0) * 0.1 + std::cos(30.0 * sgs::kPi / 180.0) * 0.3,
              1e-12);
}

TEST(Math, ShortestPath) {
  sgs::Quat q{-0.1, 0, 0, 0};
  sgs::quat_shortest_path(&q);
  EXPECT_GT(q.w, 0.0);
}

TEST(Math, HamiltonMultiply) {
  const sgs::Quat i{0, 1, 0, 0};
  const sgs::Quat j{0, 0, 1, 0};
  const sgs::Quat k{0, 0, 0, 1};
  const sgs::Quat ij = sgs::quat_mul(i, j);
  EXPECT_NEAR(ij.w, 0.0, 1e-12);
  EXPECT_NEAR(ij.x, 0.0, 1e-12);
  EXPECT_NEAR(ij.y, 0.0, 1e-12);
  EXPECT_NEAR(ij.z, 1.0, 1e-12);
  const sgs::Quat ji = sgs::quat_mul(j, i);
  EXPECT_NEAR(ji.z, -1.0, 1e-12);
  const sgs::Quat ii = sgs::quat_mul(i, i);
  EXPECT_NEAR(ii.w, -1.0, 1e-12);
  EXPECT_NEAR(ii.x, 0.0, 1e-12);
}
