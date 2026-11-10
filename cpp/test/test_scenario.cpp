// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

#include "sgs/sim/scenario.hpp"
#include "sgs/types.hpp"

namespace {

sgs::Gains g() { return sgs::default_gains(); }

}  // namespace

TEST(Scenario, PitchStepCriteria) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::StepPitch;
  sc.duration_s = 4.0;
  sgs::sim::Trace tr;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_LE(m.overshoot_frac, 0.25);
  EXPECT_LE(m.rise_s, 0.40);
  EXPECT_LE(m.max_err_settle, 0.5 * 3.14159265358979323846 / 180.0);
}

TEST(Scenario, YawStepCriteria) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::StepYaw;
  sc.duration_s = 4.0;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_LE(m.overshoot_frac, 0.25);
  EXPECT_LE(m.rise_s, 0.40);
  EXPECT_LE(m.max_err_settle, 0.5 * 3.14159265358979323846 / 180.0);
}

TEST(Scenario, ZeroGainNoTrack) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::ZeroGainStepPitch;
  sc.duration_s = 4.0;
  sc.zero_gains = true;
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(1, sc, g(), &tr, nullptr);
  double bz0 = 0.0;
  double bz1 = 0.0;
  for (const auto& s : tr.samples) {
    if (s.t >= 0.49 && s.t <= 0.51) {
      bz0 = s.boresight_err_pitch;
    }
    if (s.t >= 3.0) {
      bz1 = s.boresight_err_pitch;
    }
  }
  EXPECT_LT(std::fabs(bz1 - bz0), 0.2 * 3.14159265358979323846 / 180.0);
}

TEST(Scenario, Isolation05HzPitch) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::Isolation05Pitch;
  sc.isolation_freq_hz = 0.5;
  sc.duration_s = 12.0;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_LE(m.transmissibility, 0.35);
  sc.zero_gains = true;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_GE(m.transmissibility, 0.90);
}

TEST(Scenario, Isolation05HzYaw) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::Isolation05Yaw;
  sc.isolation_freq_hz = 0.5;
  sc.duration_s = 12.0;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_LE(m.transmissibility, 0.35);
  sc.zero_gains = true;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_GE(m.transmissibility, 0.90);
}

TEST(Scenario, Isolation10HzPitch) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::Isolation10Pitch;
  sc.isolation_freq_hz = 10.0;
  sc.duration_s = 12.0;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_LE(m.transmissibility, 0.35);
}

TEST(Scenario, Isolation10HzYaw) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::Isolation10Yaw;
  sc.isolation_freq_hz = 10.0;
  sc.duration_s = 12.0;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_LE(m.transmissibility, 0.35);
}

TEST(Scenario, CombinedRms) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::CombinedDisturbance;
  sc.perfect_sensors = false;
  sc.duration_s = 7.0;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::Trace tr;
  sgs::sim::run_scenario(2, sc, g(), &tr, &m);
  EXPECT_LE(m.rms_boresight_err, 1.5 * 3.14159265358979323846 / 180.0);
}

TEST(Scenario, ResidualRollTransmissibility) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::ResidualRoll;
  sc.duration_s = 12.0;
  sgs::sim::Trace tr;
  sgs::sim::ScenarioMetrics m{};
  sgs::sim::run_scenario(1, sc, g(), &tr, &m);
  EXPECT_GE(m.transmissibility, 0.70);
  EXPECT_LE(m.transmissibility, 1.30);
}

TEST(Scenario, Determinism) {
  sgs::sim::Scenario sc{};
  sc.kind = sgs::sim::ScenarioKind::CombinedDisturbance;
  sc.perfect_sensors = false;
  sgs::sim::Trace a, b;
  sgs::sim::run_scenario(2, sc, g(), &a, nullptr);
  sgs::sim::run_scenario(2, sc, g(), &b, nullptr);
  ASSERT_EQ(a.samples.size(), b.samples.size());
  for (size_t i = 0; i < a.samples.size(); ++i) {
    EXPECT_NEAR(a.samples[i].boresight_err_pitch, b.samples[i].boresight_err_pitch, 1e-9);
    EXPECT_NEAR(a.samples[i].boresight_err_yaw, b.samples[i].boresight_err_yaw, 1e-9);
    EXPECT_NEAR(a.samples[i].residual_roll, b.samples[i].residual_roll, 1e-9);
  }
}
