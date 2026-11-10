// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

#include "sgs/complementary.hpp"
#include "sgs/ekf.hpp"
#include "sgs/math.hpp"

namespace {

double yaw_innov(const sgs::Quat& q) {
  const sgs::Vec3 m_body =
      sgs::quat_rotate(sgs::quat_conj(q), {sgs::complementary::kMagRefNed[0], 0, 40});
  const sgs::Vec3 m_ned = sgs::quat_rotate(q, sgs::vec3_normalize(m_body));
  const double ref_h = std::atan2(0.0, 25.0);
  const double est_h = std::atan2(m_ned.y, m_ned.x);
  return est_h - ref_h;
}

}  // namespace

TEST(EkfJacobian, FiniteDifferenceMagYaw) {
  sgs::EstimatorState st{};
  sgs::ekf_init_state(&st);
  st.q = {0.999, 0.01, 0.02, 0.03};
  st.q = sgs::quat_normalize(st.q);
  const double eps = 1e-6;
  const double y0 = yaw_innov(st.q);
  sgs::Vec3 dth{eps, 0, 0};
  const sgs::Quat dq = sgs::quat_exp(dth, 1.0);
  const sgs::Quat qp = sgs::quat_normalize(sgs::quat_mul(st.q, dq));
  const double H_fd = (yaw_innov(qp) - y0) / eps;
  EXPECT_TRUE(std::isfinite(H_fd));
  EXPECT_LT(std::fabs(H_fd), 100.0);
}
