// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sgs/types.hpp"

namespace sgs {

double pid_step(double dt, const PidGains& gains, double tau_d_s, double e, double meas,
                PidState* st, double umin, double umax, double deadzone, bool freeze_integ);

void pid_reset_derivative(PidState* st);

}  // namespace sgs
