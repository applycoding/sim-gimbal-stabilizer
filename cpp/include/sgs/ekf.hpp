// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sgs/types.hpp"

namespace sgs {

// Single-state Kalman update: returns posterior x and P
double kalman_update_scalar(double* x, double* P, double z, double R, double H);

void ekf_init_state(EstimatorState* st);
void ekf_predict(double dt, const ImuSample& imu, const Gains& gains, EstimatorState* st);
void ekf_update(double dt, const ImuSample& imu, const Gains& gains, const Limits& limits,
                EstimatorState* st, uint32_t* flags);

}  // namespace sgs
