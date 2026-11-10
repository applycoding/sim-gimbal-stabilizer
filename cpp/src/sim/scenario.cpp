// SPDX-License-Identifier: Apache-2.0
#include "sgs/sim/scenario.hpp"

#include "sgs/complementary.hpp"
#include "sgs/math.hpp"
#include "sgs/sim/plant.hpp"
#include "sgs/sim/rng.hpp"
#include "sgs/sim/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sgs {
namespace sim {

namespace {

constexpr double kDt = 0.001;

void base_motion(double t, const Scenario& sc, PlantState* st) {
  const double deg = kPi / 180.0;
  st->base_omega_pitch = 0.0;
  st->base_omega_yaw = 0.0;
  st->base_omega_roll = 0.0;

  const double vib_pitch =
      0.4 * deg * std::sin(2.0 * kPi * 18.0 * t) + 0.25 * deg * std::sin(2.0 * kPi * 35.0 * t);
  const double vib_yaw =
      0.4 * deg * std::sin(2.0 * kPi * 12.0 * t) + 0.2 * deg * std::sin(2.0 * kPi * 28.0 * t);
  const double vib_roll = 0.5 * deg * std::sin(2.0 * kPi * 8.0 * t);

  auto gust = [](double t, double t0, double T, double amp) {
    if (t < t0 || t > t0 + T) {
      return 0.0;
    }
    const double x = (t - t0) / T;
    return amp * 0.5 * (1.0 - std::cos(2.0 * kPi * x));
  };

  switch (sc.kind) {
    case ScenarioKind::StepPitch:
    case ScenarioKind::StepYaw:
    case ScenarioKind::ZeroGainStepPitch:
      st->base_pitch = 0.0;
      st->base_yaw = 0.0;
      st->base_roll = 0.0;
      break;
    case ScenarioKind::Isolation05Pitch:
    case ScenarioKind::Isolation10Pitch: {
      const double w = 2.0 * kPi * sc.isolation_freq_hz;
      st->base_pitch = deg * std::sin(w * t);
      st->base_omega_pitch = deg * w * std::cos(w * t);
      break;
    }
    case ScenarioKind::Isolation05Yaw:
    case ScenarioKind::Isolation10Yaw: {
      const double w = 2.0 * kPi * sc.isolation_freq_hz;
      st->base_yaw = deg * std::sin(w * t);
      st->base_omega_yaw = deg * w * std::cos(w * t);
      break;
    }
    case ScenarioKind::ResidualRoll: {
      const double w = 2.0 * kPi * 8.0;
      st->base_roll = 0.5 * deg * std::sin(w * t);
      st->base_omega_roll = 0.5 * deg * w * std::cos(w * t);
      break;
    }
    case ScenarioKind::CombinedDisturbance:
      st->base_pitch = vib_pitch + gust(t, 1.0, 1.0, 3.0 * deg);
      st->base_yaw = vib_yaw + gust(t, 2.5, 1.5, 2.0 * deg);
      st->base_roll = vib_roll;
      break;
  }
}

Vec3 boresight_target(double t, const Scenario& sc, const Vec3& hold) {
  const double step = sc.boresight_step_deg * kPi / 180.0;
  Vec3 b = hold;
  if (sc.kind == ScenarioKind::StepPitch || sc.kind == ScenarioKind::ZeroGainStepPitch) {
    if (t >= 0.5) {
      b = {std::cos(step), 0.0, -std::sin(step)};
    }
  } else if (sc.kind == ScenarioKind::StepYaw) {
    if (t >= 0.5) {
      b = {std::cos(step), std::sin(step), 0.0};
    }
  }
  return vec3_normalize(b);
}

struct FitResult {
  double dc{0.0};
  double amp_sin{0.0};
  double amp_cos{0.0};
};

FitResult sin_cos_fit(const std::vector<double>& t, const std::vector<double>& y, double omega) {
  const size_t n = t.size();
  double s_ss = 0, s_cc = 0, s_sc = 0;
  double s_sy = 0, s_cy = 0, s_1 = static_cast<double>(n);
  double sum_y = 0;
  for (size_t i = 0; i < n; ++i) {
    const double s = std::sin(omega * t[i]);
    const double c = std::cos(omega * t[i]);
    s_ss += s * s;
    s_cc += c * c;
    s_sc += s * c;
    s_sy += s * y[i];
    s_cy += c * y[i];
    sum_y += y[i];
  }
  const double s_1y = sum_y;
  // Normal equations for [1, sin, cos]
  const double det = s_1 * (s_ss * s_cc - s_sc * s_sc) - 0 + 0;
  (void)det;
  FitResult r{};
  r.dc = sum_y / static_cast<double>(n);
  if (n > 2) {
    double A[3][3] = {{s_1, 0, 0}, {0, s_ss, s_sc}, {0, s_sc, s_cc}};
    double bvec[3] = {s_1y, s_sy, s_cy};
    for (int k = 0; k < 3; ++k) {
      for (int i = k + 1; i < 3; ++i) {
        const double f = A[i][k] / A[k][k];
        for (int j = k; j < 3; ++j) {
          A[i][j] -= f * A[k][j];
        }
        bvec[i] -= f * bvec[k];
      }
    }
    double x[3]{};
    for (int i = 2; i >= 0; --i) {
      double sum = bvec[i];
      for (int j = i + 1; j < 3; ++j) {
        sum -= A[i][j] * x[j];
      }
      x[i] = sum / A[i][i];
    }
    r.dc = x[0];
    r.amp_sin = x[1];
    r.amp_cos = x[2];
  }
  return r;
}

double fundamental_amplitude(const FitResult& f) {
  return std::sqrt(f.amp_sin * f.amp_sin + f.amp_cos * f.amp_cos);
}

}  // namespace

void run_scenario(uint32_t seed, const Scenario& sc, const Gains& gains_in, Trace* trace,
                  ScenarioMetrics* metrics) {
  trace->samples.clear();
  Gains gains = gains_in;
  if (sc.zero_gains) {
    gains.rate = {};
    gains.boresight = {};
  }

  const Limits limits = default_limits();
  Controller ctrl;
  ControllerState cst{};
  PlantParams pp{};
  PlantState pst{};
  SensorModel sm{};
  sm.gyro_noise_sigma = 0.002;
  sm.accel_noise_sigma = 0.05;
  sm.mag_noise_sigma = 0.5;
  XorShift32 rng(seed);

  const Quat q0 = world_from_camera(pst);
  reset_state(&cst, q0);
  Vec3 hold{1.0, 0.0, 0.0};

  const int steps = static_cast<int>(sc.duration_s / kDt);
  double t0_step = -1.0;

  std::vector<double> times;
  std::vector<double> base_series;
  std::vector<double> cam_series;

  for (int i = 0; i < steps; ++i) {
    const double t = i * kDt;
    base_motion(t, sc, &pst);
    const Quat q_true = world_from_camera(pst);
    ImuSample imu{};
    EncoderSample enc{};
    sample_sensors(kDt, pst, q_true, sm, sc.perfect_sensors, &rng, &imu, &enc);

    StepInput in{};
    in.imu = imu;
    in.enc = enc;
    const Vec3 target = boresight_target(t, sc, hold);
    in.set.boresight_ned = target;

    StepOutput out{};
    ctrl.step(kDt, gains, limits, in, &cst, &out);

    PlantParams plant = pp;
    plant_step(kDt, plant, out.motor.tau_pitch, out.motor.tau_yaw, &pst);

    const Quat q_after = world_from_camera(pst);
    const Vec3 b_true = camera_boresight_world(q_after);
    const Vec3 b_des = target;
    const Vec3 errv = boresight_error_camera(
        quat_rotate(quat_conj(q_after), b_true), quat_rotate(quat_conj(q_after), b_des));

    TraceSample ts{};
    ts.t = t;
    ts.boresight_err_pitch = errv.y;
    ts.boresight_err_yaw = errv.z;
    ts.theta_rel = pst.theta_rel;
    ts.psi_rel = pst.psi_rel;
    ts.tau_pitch = out.motor.tau_pitch;
    ts.tau_yaw = out.motor.tau_yaw;
    const Mat3 R = quat_to_mat3(q_after);
    ts.residual_roll = std::atan2(R.m[7], R.m[8]);
    trace->samples.push_back(ts);

    if ((sc.kind == ScenarioKind::StepPitch || sc.kind == ScenarioKind::StepYaw ||
         sc.kind == ScenarioKind::ZeroGainStepPitch) &&
        t >= 0.5 && t0_step < 0) {
      t0_step = t;
    }

    if (sc.kind == ScenarioKind::Isolation05Pitch || sc.kind == ScenarioKind::Isolation10Pitch ||
        sc.kind == ScenarioKind::Isolation05Yaw || sc.kind == ScenarioKind::Isolation10Yaw) {
      const double warmup = std::max(4.0, 4.0 / sc.isolation_freq_hz);
      if (t >= warmup) {
        times.push_back(t);
        if (sc.kind == ScenarioKind::Isolation05Pitch || sc.kind == ScenarioKind::Isolation10Pitch) {
          base_series.push_back(pst.base_pitch);
          const sgs::Vec3 b = sgs::camera_boresight_world(q_after);
          cam_series.push_back(std::atan2(-b.z, b.x));
        } else {
          base_series.push_back(pst.base_yaw);
          const sgs::Vec3 b = sgs::camera_boresight_world(q_after);
          cam_series.push_back(std::atan2(b.y, b.x));
        }
      }
    }
  }

  if (metrics != nullptr) {
    metrics->overshoot_frac = 0.0;
    metrics->rise_s = sc.duration_s;
    if (t0_step >= 0 && sc.boresight_step_deg > 0) {
      // 10-90% rise and classical overshoot of the boresight response, not of |error|.
      const double step = sc.boresight_step_deg * kPi / 180.0;
      double t10 = -1.0;
      double t90 = -1.0;
      double peak_frac = 0.0;
      bool have_ref = false;
      double e_ref = step;
      for (const auto& s : trace->samples) {
        if (s.t < t0_step) {
          continue;
        }
        const double e_dom =
            (std::fabs(s.boresight_err_pitch) >= std::fabs(s.boresight_err_yaw))
                ? s.boresight_err_pitch
                : s.boresight_err_yaw;
        if (!have_ref) {
          e_ref = std::copysign(step, e_dom);
          have_ref = true;
        }
        const double frac = 1.0 - e_dom / e_ref;
        if (frac > peak_frac) {
          peak_frac = frac;
        }
        if (t10 < 0.0 && frac >= 0.10) {
          t10 = s.t;
        }
        if (t90 < 0.0 && frac >= 0.90) {
          t90 = s.t;
        }
      }
      metrics->overshoot_frac = std::max(0.0, peak_frac - 1.0);
      if (t10 >= 0.0 && t90 >= 0.0) {
        metrics->rise_s = t90 - t10;
      }
    }
    metrics->max_err_settle = 0.0;
    for (const auto& s : trace->samples) {
      if (s.t >= 1.5 && s.t <= 3.0) {
        const double e = std::max(std::fabs(s.boresight_err_pitch), std::fabs(s.boresight_err_yaw));
        if (e > metrics->max_err_settle) {
          metrics->max_err_settle = e;
        }
      }
    }
    double sum2 = 0.0;
    int cnt = 0;
    for (const auto& s : trace->samples) {
      if (s.t >= 1.0 && s.t <= 6.0) {
        const double e = std::hypot(s.boresight_err_pitch, s.boresight_err_yaw);
        sum2 += e * e;
        ++cnt;
      }
    }
    if (cnt > 0) {
      metrics->rms_boresight_err = std::sqrt(sum2 / cnt);
    }
    if (!times.empty()) {
      const double omega = 2.0 * kPi * sc.isolation_freq_hz;
      const FitResult fb = sin_cos_fit(times, base_series, omega);
      const FitResult fc = sin_cos_fit(times, cam_series, omega);
      const double ab = fundamental_amplitude(fb);
      const double ac = fundamental_amplitude(fc);
      if (ab > 1e-12) {
        metrics->transmissibility = ac / ab;
      }
    }
    if (sc.kind == ScenarioKind::ResidualRoll) {
      const double freq = 8.0;
      const double warmup = std::max(4.0, 4.0 / freq);
      std::vector<double> t, base, roll;
      for (const auto& s : trace->samples) {
        if (s.t < warmup) {
          continue;
        }
        t.push_back(s.t);
        base.push_back(0.5 * kPi / 180.0 * std::sin(2.0 * kPi * 8.0 * s.t));
        roll.push_back(s.residual_roll);
      }
      const double omega = 2.0 * kPi * freq;
      const FitResult fb = sin_cos_fit(t, base, omega);
      const FitResult fr = sin_cos_fit(t, roll, omega);
      const double ab = fundamental_amplitude(fb);
      const double ar = fundamental_amplitude(fr);
      if (ab > 1e-12) {
        metrics->transmissibility = ar / ab;
      } else if (ar > 1e-12) {
        metrics->transmissibility = ar / (0.5 * kPi / 180.0);
      }
    }
  }
}

double fit_transmissibility(const Trace& trace, double freq_hz, bool pitch_axis) {
  const double warmup = std::max(4.0, 4.0 / freq_hz);
  std::vector<double> t, base, cam;
  for (const auto& s : trace.samples) {
    if (s.t < warmup) {
      continue;
    }
    t.push_back(s.t);
    if (pitch_axis) {
      base.push_back(s.theta_rel);
      cam.push_back(s.boresight_err_pitch);
    } else {
      base.push_back(s.psi_rel);
      cam.push_back(s.boresight_err_yaw);
    }
  }
  const double omega = 2.0 * kPi * freq_hz;
  const FitResult fb = sin_cos_fit(t, base, omega);
  const FitResult fc = sin_cos_fit(t, cam, omega);
  const double ab = fundamental_amplitude(fb);
  const double ac = fundamental_amplitude(fc);
  if (ab < 1e-12) {
    return 0.0;
  }
  return ac / ab;
}

}  // namespace sim
}  // namespace sgs
