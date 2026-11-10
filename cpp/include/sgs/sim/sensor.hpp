// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sgs/sim/plant.hpp"
#include "sgs/sim/rng.hpp"
#include "sgs/types.hpp"

namespace sgs {
namespace sim {

struct SensorModel {
  double gyro_bias[3]{};
  double accel_bias[3]{};
  double gyro_noise_sigma{0.0};
  double accel_noise_sigma{0.0};
  double mag_noise_sigma{0.0};
  bool mag_enabled{true};
};

void sample_sensors(double dt, const PlantState& plant, const Quat& q_world_from_camera,
                    const SensorModel& model, bool perfect, XorShift32* rng, ImuSample* imu,
                    EncoderSample* enc);

}  // namespace sim
}  // namespace sgs
