// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sgs/types.hpp"

namespace sgs {

namespace complementary {
constexpr double kMagRefNed[3] = {25.0, 0.0, 40.0};
}

void complementary_predict(double dt, const ImuSample& imu, double comp_kp_acc,
                           double comp_kp_mag, EstimatorState* st, uint32_t* flags);

// Heading innovation (rad) from magnetometer versus reference field; body-frame measurement.
double mag_heading_error_rad(const Quat& q, const ImuSample& imu);

}  // namespace sgs
