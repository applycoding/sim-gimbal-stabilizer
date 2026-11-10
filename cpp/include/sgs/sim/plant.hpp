// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sgs/types.hpp"

namespace sgs {
namespace sim {

struct PlantParams {
  double J{1.5e-4};
  double b{2.0e-4};
  double tau_c{1.0e-3};
  double k_spring{0.02};
  double tau_max{0.15};
  double deadzone{0.001};
  double theta_stop{80.0 * 3.14159265358979323846 / 180.0};
};

struct PlantState {
  double theta_rel{0.0};
  double psi_rel{0.0};
  double omega_theta{0.0};
  double omega_psi{0.0};
  double base_pitch{0.0};
  double base_yaw{0.0};
  double base_roll{0.0};
  double base_omega_pitch{0.0};
  double base_omega_yaw{0.0};
  double base_omega_roll{0.0};
};

void plant_step(double dt, const PlantParams& p, double tau_pitch_cmd, double tau_yaw_cmd,
                PlantState* st);

Quat world_from_camera(const PlantState& st);

}  // namespace sim
}  // namespace sgs
