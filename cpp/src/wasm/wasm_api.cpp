// SPDX-License-Identifier: Apache-2.0
#include "sgs/types.hpp"

#include "sgs/math.hpp"
#include "sgs/sim/plant.hpp"
#include "sgs/sim/rng.hpp"
#include "sgs/sim/sensor.hpp"

#include <cmath>

namespace {

sgs::Controller g_ctrl;
sgs::ControllerState g_state{};
sgs::Gains g_gains = sgs::default_gains();
sgs::Limits g_limits = sgs::default_limits();
sgs::sim::PlantParams g_pp{};
sgs::sim::PlantState g_pst{};
sgs::sim::SensorModel g_sm{};
sgs::sim::XorShift32 g_rng{1};
sgs::Vec3 g_boresight{1.0, 0.0, 0.0};
int g_step = 0;
bool g_step_armed = false;

constexpr int kTraceFloats = 5;
constexpr int kTraceCap = 8192;
alignas(8) float g_trace[kTraceCap * kTraceFloats]{};
int g_trace_len = 0;

}  // namespace

extern "C" {

int sgs_trace_stride() { return kTraceFloats; }
int sgs_trace_capacity() { return kTraceCap; }
float* sgs_trace_ptr() { return g_trace; }
int sgs_trace_len() { return g_trace_len; }

void sgs_reset() {
  g_pst = {};
  const sgs::Quat q0 = sgs::sim::world_from_camera(g_pst);
  sgs::reset_state(&g_state, q0);
  g_gains = sgs::default_gains();
  g_boresight = {1.0, 0.0, 0.0};
  g_trace_len = 0;
  g_step = 0;
  g_step_armed = false;
}

void sgs_set_gains(float rate_kp, float rate_ki, float att_kp, float att_ki) {
  g_gains.rate.kp = rate_kp;
  g_gains.rate.ki = rate_ki;
  g_gains.boresight.kp = att_kp;
  g_gains.boresight.ki = att_ki;
}

void sgs_set_estimator(int use_ekf) {
  g_state.select = use_ekf ? sgs::ControllerState::EstimatorSelect::Ekf
                           : sgs::ControllerState::EstimatorSelect::Complementary;
}

void sgs_pitch_step() { g_step_armed = true; }

int sgs_step_ms(int ms) {
  if (ms < 0) {
    ms = 0;
  }
  if (ms > 50) {
    ms = 50;
  }
  for (int i = 0; i < ms; ++i) {
    const double t = g_step * 0.001;
    if (g_step_armed && g_step >= 500) {
      const double step = 10.0 * sgs::kPi / 180.0;
      g_boresight = {std::cos(step), 0.0, -std::sin(step)};
    }
    const sgs::Quat q_true = sgs::sim::world_from_camera(g_pst);
    sgs::ImuSample imu{};
    sgs::EncoderSample enc{};
    sgs::sim::sample_sensors(0.001, g_pst, q_true, g_sm, true, &g_rng, &imu, &enc);

    sgs::StepInput in{};
    in.imu = imu;
    in.enc = enc;
    in.set.boresight_ned = g_boresight;
    sgs::StepOutput out{};
    g_ctrl.step(0.001, g_gains, g_limits, in, &g_state, &out);
    sgs::sim::plant_step(0.001, g_pp, out.motor.tau_pitch, out.motor.tau_yaw, &g_pst);
    if (g_trace_len < kTraceCap) {
      const int o = g_trace_len * kTraceFloats;
      g_trace[o + 0] = static_cast<float>(t);
      g_trace[o + 1] = static_cast<float>(out.boresight_error_rad.y);
      g_trace[o + 2] = static_cast<float>(out.boresight_error_rad.z);
      g_trace[o + 3] = static_cast<float>(out.motor.tau_pitch);
      g_trace[o + 4] = static_cast<float>(out.motor.tau_yaw);
      ++g_trace_len;
    }
    ++g_step;
  }
  return g_trace_len;
}

}  // extern "C"
