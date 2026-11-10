// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <vector>

#include "sgs/types.hpp"

namespace sgs {
namespace sim {

enum class ScenarioKind {
  StepPitch,
  StepYaw,
  Isolation05Pitch,
  Isolation05Yaw,
  Isolation10Pitch,
  Isolation10Yaw,
  ResidualRoll,
  CombinedDisturbance,
  ZeroGainStepPitch,
};

struct Scenario {
  ScenarioKind kind{ScenarioKind::StepPitch};
  double duration_s{5.0};
  double boresight_step_deg{10.0};
  bool zero_gains{false};
  bool perfect_sensors{true};
  double isolation_freq_hz{0.5};
};

struct TraceSample {
  double t{0.0};
  double boresight_err_pitch{0.0};
  double boresight_err_yaw{0.0};
  double theta_rel{0.0};
  double psi_rel{0.0};
  double tau_pitch{0.0};
  double tau_yaw{0.0};
  double residual_roll{0.0};
};

struct Trace {
  std::vector<TraceSample> samples;
};

struct ScenarioMetrics {
  double overshoot_frac{0.0};
  double rise_s{0.0};
  double max_err_settle{0.0};
  double transmissibility{0.0};
  double rms_boresight_err{0.0};
};

void run_scenario(uint32_t seed, const Scenario& sc, const Gains& gains, Trace* trace,
                  ScenarioMetrics* metrics = nullptr);

double fit_transmissibility(const Trace& trace, double freq_hz, bool pitch_axis);

}  // namespace sim
}  // namespace sgs
