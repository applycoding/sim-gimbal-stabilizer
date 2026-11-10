# sim-gimbal-stabilizer — v1 plan

Revised 2026-11-10 after independent version, URL, license, and control-loop checks. Outbound license is **Apache-2.0**, not MIT.

## Goal

Public portfolio demo for Arpan Bom, aimed at Harmattan AI's Software Engineer - Gimbal role (Paris, on-site; https://jobs.ashbyhq.com/harmattan-ai/0faad309-78ba-4c0b-a67e-4dcaea72f73d). The role owns real-time gimbal control and GNC on custom MCUs and embedded Linux: embedded C/C++, quaternions, attitude, estimation, telemetry, and motor/IMU I/O, with drone-autopilot familiarity expected.

The demo is a **simulated** two-axis (pitch and yaw) camera gimbal that holds a world-frame **boresight** (optical-axis) setpoint while a scripted base shakes. A fixed-step C++17 core runs a cascaded PID (inner **inertial-rate** loop, outer boresight loop), a complementary filter, and a 6-state error-state EKF on a noisy gyro, accelerometer, and magnetometer. Host code adds a torque plant with saturation, a command dead-zone, vibration, and 1-cosine gusts. The Pages UI plots live boresight error, a step response, and pitch/yaw transmissibility.

This is a **portfolio demo, simulation only**. Do not claim real hardware, flight tests, production gimbal firmware, certification, or that Arpan has owned a production gimbal controller.

Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

## Scope

One camera on a moving base. v1 actuates yaw about the base z-axis, then pitch about the yawed y-axis. Roll actuation is a stretch goal and is not in the v1 API. The controlled objective is boresight pointing; residual roll about the optical axis is unactuated and is plotted honestly.

- Core (`sgs_core`): POD math, joint-to-camera allocation, PID, complementary filter, 6-state error-state EKF, `Controller::step`. No plant, no ROS, no JSON, no heap.
- Sim (`sgs_sim`, host and WASM only): simplified per-joint plant, sensor model, scripted base motion, scenario runner. Not part of the MCU-style no-heap gate.
- Tests: GoogleTest. A thin pybind11 module is optional and off by default. Core ctest does not need Python or ROS.
- Browser: the same C++ core and sim compiled to WASM. TypeScript draws sliders and Canvas 2D plots only. No second PID or EKF.
- ROS 2 Humble wrapper is optional and off by default.

Do not duplicate applycoding repos. `cpp-flight-controller` (MIT) is a C++20 **quadrotor** attitude/rate stack with a Quad-X mixer, altitude KF, and rigid-body quad. This repo is a camera on a base: no mixer, no altitude loop, no quad plant. Reimplement quaternions, PID, complementary filter, and EKF from first principles; do not copy that tree. `embedded-motor-controller` (MIT) is a single-axis DC **speed** loop with a UART FSM and HAL stubs. A torque plant with saturation and a dead-zone is in scope; a UART protocol, fault FSM, or watchdog is not. `rust-drone-failsafe` (MIT) is a Rust failsafe supervisor. Its WASM-and-Pages layout is a process idea only. Nav2 coverage, PX4 mission planning, warehouse CBS, and the drone mission sim are unrelated. Do not depend on them.

## Architecture

**Decision:** two static libraries. `sgs_core` is the interview artifact and is what an MCU would compile. `sgs_sim` owns physics and noise and links only into tests, pybind11, the ROS node, and WASM. The browser imports one WASM module built from those libraries. The ROS node is an adapter around `sgs_sim`.

**Why:** Harmattan’s loop is embedded C/C++. A TypeScript controller would not be that artifact. Splitting the plant keeps `<vector>` trace buffers out of the core so `-fno-exceptions -fno-rtti` and the include lint stay honest. One sample rate avoids a multi-rate decimator on a first MCU port. Humble matches the other portfolio demos and is supported until May 2027; it is not claimed as the current ROS LTS. Jazzy is a later port, not a v1 target.

### Frames and kinematics

World is NED (x north, y east, z down). Base and camera are FRD (x forward / optical axis, y right, z down). With both joint angles at 0, the camera axes match the base. Hamilton quaternions, `w + xi + yj + zk`, right-handed, active: `v_parent = R(q_parent_from_child) * v_child`. Product order is parent-from-child on the right:

`q_world_from_camera = q_world_from_base ⊗ q_yaw(ψ) ⊗ q_pitch(θ)`.

`q_yaw(ψ) = (cos(ψ/2), 0, 0, sin(ψ/2))` rotates about base +z. `q_pitch(θ) = (cos(θ/2), 0, sin(θ/2), 0)` rotates about the yawed +y. **Positive θ rotates the optical axis toward world/base −z, which is upward in FRD** (the +x basis becomes `(cos θ, 0, −sin θ)`). The draft’s “downward” sentence was wrong; tests must rotate +90° and check that basis, not only multiply/normalize. Shortest-path quaternion sign: if `q.w < 0`, negate `q`. The tie `w == 0` uses a documented rule (prefer `x > 0`, then `y`, then `z`).

### Boresight objective (not full 3-axis attitude)

Two actuators cannot command an arbitrary world quaternion once the base has roll. v1 tracks the camera **optical axis** `b = +x`. Roll about `b` is residual and free. The outer-loop error in the camera frame is `e = b_est × b_des` (unit vectors). The y component is the pitch error and the z component is the yaw/pan error. A 10° pitch step means a 10° boresight step. Do not claim 3-axis isolation or full-attitude optimality. Unreachable full-attitude setpoints are projected onto the boresight objective before the PID sees them.

### Inner loop: inertial rate, measured from the camera gyro

The camera gyro measures **inertial** camera rate, not joint rate:

`ω_cam = ω_base_in_camera + A(θ) q̇ + bias + noise`,

with `A(θ) = [[0, −sin θ], [1, 0], [0, cos θ]]` (columns: pitch, yaw). v1 is an **inertial-rate** cascade, which fits “no base IMU”:

- Outer loop (boresight error) emits a camera-frame inertial rate setpoint `ω_sp`. The roll (x) command is not independently controlled.
- Measurements use the bias-corrected gyro of the **selected** estimator (complementary bias is identically 0). The least-squares joint-coordinate inertial rates are `θ̇_meas = ω_y` and `ψ̇_meas = −sin θ · ω_x + cos θ · ω_z`. Do not label `gyro_z / cos θ` as a joint rate, and do not drop the `ω_x` term.
- Inner PIDs track those two rates and emit joint torques. With `m` the camera-frame effort from the rate PIDs (`m_x = 0`), `τ_pitch = m_y` and `τ_yaw = −sin θ · m_x + cos θ · m_z`.
- A yaw joint command at nonzero pitch also produces a camera-x (roll) rate. That leftover is expected and plotted.

There is **no base-rate feed-forward in v1** and no base IMU. Gyro feedback already sees base motion. A `ff_base` field is not part of the v1 API (adding the wrong sign was a draft hazard).

### Singularity and limits

When `|θ| > 75°`, yaw about base z is mostly roll about the optical axis (reduced pan authority). Set `near_singularity`, freeze the yaw integrator, and keep commands finite. There is no `1/cos θ` divisor in the LS map, so there is nothing to floor. `sign(cos)` is unused. The plant clamps `|θ|` to 80° and **zeroes outward** `ω_rel` at the stop (no energy injection). A core unit test may inject an encoder sample at 85° (legal for the controller even though the plant cannot reach it) and expect the flag and finite torques. Recovery from the 80° stop back to mid-range is also tested.

### Sensors and estimators

Strapped-down camera IMU in `sgs_sim`, at an **ideal rotation center** (no lever-arm or centripetal model). Gyro and accel carry a constant bias, per-sample white noise, and an optional bias random walk (default walk sigma 0 in the **plant**; the EKF still has a small process walk so bias is a random constant / slow walk). A magnetometer makes yaw/heading observable. **Accelerometer gravity does not observe yaw.** An accel-only complementary filter must not be described as holding heading. Encoders are 12-bit over `[-π, π)` plus ±0.5 LSB uniform noise, with wrap documented.

At rest in NED/FRD, specific force is `f = R^T (a_world − g_ned)` with `g_ned = [0, 0, +9.80665]`, so a level camera reports `[0, 0, −g]`. State that in the README and in comments.

**Complementary filter (Mahony-style proportional correction, no bias state):** integrate `ω = gyro` (uncorrected), then correct with `ω_corr = k_p,acc (f̂ × f_pred) + k_p,mag m_yaw_err`. Defaults: `comp_kp_acc = 2.0 /s`, `comp_kp_mag = 0.5 /s`. It does **not** estimate gyro bias. Static 20° pitch, zero bias: `|pitch error| < 1°` by 3 s. A second case with a constant gyro bias of `0.01 rad/s` must match the predicted small-angle equilibrium `bias / comp_kp_acc ≈ 0.005 rad` (about 0.29°) within `0.15°`, and must **not** be required to go to zero.

**Error-state EKF:** 6 states `[δθx, δθy, δθz, b_gx, b_gy, b_gz]`, `P` a row-major 6×6, error-state injection then reset, Joseph-form covariance update (or an equivalent stable form), symmetry / PSD sanity. Defaults: `ekf_sigma_gyro = 0.01 rad/s`, `ekf_sigma_bias_rw = 1e-4 rad/s/√s`, `ekf_sigma_acc = 0.3 m/s²`, `ekf_sigma_mag = 2 µT`. Discrete process noise scales with `dt`. Accel updates are skipped when `abs(||a|| − g) / g > 0.15` or an accel direction innovation exceeds 25°, and `accel_gated` is set. The magnitude gate **does not** reject all translation (a 0.5 g horizontal accel changes the norm by only about 12% and can still tilt the estimate); the README says so. Mag updates are skipped when the magnitude gate `> 0.30`, the direction innovation `> 30°`, the sample lacks `mag_valid`, or the estimated horizontal field is below 5 µT (`mag_weak`). A mag-off (or vertical-only field) test must show yaw bias is **not** silently observed. A finite-difference check of the measurement Jacobian is required. The 1-state update routine used inside the filter has a hand example: `P = 1`, `R = 1`, `z = 1`, `x = 0` yields `K = 0.5`, `x = 0.5`, `P = 0.5` within `1e-12`. Static bench with mag on: constant yaw bias `0.01 rad/s`, 8 s, `|bias error| < 0.003 rad/s` and boresight error `< 1°`.

The UI can select complementary, EKF, or overlay (overlay is display-only; **one** selected estimator closes the loop).

### Plant (sim only) — teaching approximation

Per joint, a **scalar** teaching model, not a full spatial multi-body derivation:

`J * α_inertial_axis = τ − b ω_rel − τ_c tanh(ω_rel / 0.05) − k_spring q_rel`,

with `α_inertial_axis ≈ α_base_along_that_joint_axis + α_rel`. There is no Coriolis / inter-axis inertia coupling in v1. The README calls this a simplified per-joint model. Frozen numbers:

`J = 1.5e-4 kg·m²`, `b = 2.0e-4 N·m·s/rad`, `τ_c = 1.0e-3 N·m`, `k_spring = 0.02 N·m/rad` (soft cable-like centering, rest angle 0), `τ_max = 0.15 N·m`, command dead-zone `0.001 N·m` (inside the zone the applied command is 0; this is below the 10° spring load of `0.0035 N·m` so a static hold is possible). Semi-implicit Euler, `dt = 0.001 s`. Hard-stop as above. The command is torque, not voltage. This is **not** FOC, SVPWM, or a BLDC driver.

These plant numbers are physically motivated (light centering, modest Coulomb, inertia typical of a small camera) and are **not** retuned to make open-loop 10 Hz isolation look bad. Linear frictionless transmissibility is `H(s) = (b s + k) / (J s² + b s + k)`, `f_n ≈ 1.84 Hz`, `|H(j 2π 10)| ≈ 0.041`. A soft gimbal already isolates at 10 Hz; that is a feature, not a test to defeat.

No Dryden model. Vibration is the sum of sinusoids plus a seeded first-order low-pass of white noise (pole 40 Hz): pitch 0.4° at 18 Hz and 0.25° at 35 Hz; yaw 0.4° at 12 Hz and 0.2° at 28 Hz; roll 0.5° at 8 Hz. The exact noise scale and xorshift-to-Gaussian method are locked in `fixtures/README.md` when goldens are written (the draft’s “about 0.05° RMS” is not enough to reproduce a golden). Gusts are 1-cosine: pitch amplitude 3°, duration 1.0 s, start 1.0 s; yaw amplitude 2°, duration 1.5 s, start 2.5 s. The README calls them documented 1-cosine gusts, not certified turbulence.

### Sample rates and PID law

Plant, rate loop, angle loop, and both estimators run at **1 kHz** with the caller’s `dt`. Tests use `0.001 s`. The core accepts `dt` in `(0, 0.002]`; anything outside returns `InvalidDt`, **zeroes torques**, initialises outputs, and **does not change state**. The core never reads a wall clock. The 0.001 s stability result is not inherited by a 50 ms step; v1 does not accept that dt.

Explicit discrete PID (back-calculation anti-windup, derivative on measurement):

```text
k_aw = k_i / k_p if k_p != 0 else 0          # one tracking-time choice, documented
α = dt / (dt + τ_d)                          # τ_d = 0.005 s
d_raw = (meas - meas_prev) / dt              # first sample: d_raw = 0, has_meas becomes true
d_filt += α * (d_raw - d_filt)
u_raw = k_p * e + integ - k_d * d_filt
u_sat = clip(u_raw, umin, umax)
u_applied = 0 if |u_sat| < deadzone else u_sat   # inner loop only; outer has no dead-zone
integ += dt * (k_i * e + k_aw * (u_applied - u_raw))
```

Outer P/I/D on boresight pitch and yaw error produce `ω_sp` with each component `|ω| ≤ 2 rad/s`. When the rate limit saturates, the outer integrator uses the same back-calculation on the limited `ω`. Roll outer gains are 0.

**Locked default gains** (the set that passed the independent one-axis feasibility checks; freeze in `defaults.hpp` and the README):

- Rate: `k_p = 0.05`, `k_i = 0.40`, `k_d = 2e-4`
- Attitude/boresight: `k_p = 10`, `k_i = 2.0`, `k_d = 0.10`

If a later coupled two-axis implementation misses a bound, change gains or document a requirement change in the same commit. Do not stiffen `k_spring` merely to make a 10 Hz open-loop guard fail. Do not tighten written bounds after the fact.

### Frequency response, step, and residual roll

Transmissibility is the ratio of fundamental amplitudes from a sin/cos + DC fit over a window of `max(4 s, 4 / f)` after a warmup of `max(4 s, 4 / f)`, at 1° base angle, pitch and yaw tested separately, setpoint held at the pre-sweep boresight. This is a nonlinear, amplitude-specific number, not a claim of a full LTI Bode plot. The demo may still plot 0.5, 1, 2, 5, 10, 20, 40 Hz. Pass/fail points:

- **0.5 Hz (feedback proof):** closed-loop ratio `≤ 0.35`; zero-gain open-loop ratio `≥ 0.90` on the same case. Independent one-axis check with the locked gains: OL `1.007`, CL `0.204`.
- **10 Hz (high-frequency isolation):** closed-loop ratio `≤ 0.35`. Open-loop on this plant is already about `0.21` (Coulomb time-sim) / `0.041` (linear). The README states that 10 Hz isolation is largely passive inertia; the 0.5 Hz comparison is the proof that feedback does work. **Do not** require open-loop `≥ 0.70` at 10 Hz. 40 Hz is plotted, not gated (control can slightly worsen very high-frequency isolation).

Step: 10° pitch boresight, then a separate 10° yaw, base fixed, perfect sensors. Overshoot `≤ 25%`, 10–90% rise `≤ 0.40 s`, inside `±0.5°` on **every sample** from 1.5 s through 3.0 s. Zero-gain on the same step must remain within `0.2°` of the start (cannot track). Independent check with locked gains: overshoot `2.0%`, rise `0.236 s`, max error `0.19°`, zero-gain stays at 0.

Combined vibration plus gusts, seeded sensor noise on, selected estimator EKF (a complementary run is reported too): RMS of **truth boresight error versus the held target** (not absolute attitude) `≤ 1.5°` over `t ∈ [1, 6]` s. Independent one-axis noisy check was `0.047°` RMS; the bound is conservative.

**Residual roll test (separate, not a combined-run lower bound):** base roll only, `0.5°` at 8 Hz, level boresight hold, no roll actuator. Residual roll is the rotation about the true optical axis. Transmissibility must lie in `[0.70, 1.30]` (the disturbance is not rejected). Do not turn expected leakage into a marketing RMS band on a mixed-disturbance run.

### No-heap and RNG

Core TUs use `-fno-exceptions -fno-rtti`. `tools/check_no_heap.py` is a **token-aware lint** with a documented allowlist (to avoid matching `new` inside comments or identifiers). It fails if `cpp/include/sgs` or `cpp/src/core` include `<vector>`, `<string>`, `<memory>`, or contain `new`, `malloc`, `calloc`, `realloc`, `std::vector`, or `std::string`. A separate executable `noheap_link` (not a gtest binary) builds only core objects, replaces `operator new/new[]` (including nothrow/aligned if used) and wraps `malloc`/`calloc`/`realloc` with abort-or-counter hooks, then runs `reset` + `step` on a static bench covering both estimators, gating, saturation, and the 85° flag. Accidental allocation fails the test. The lint is not a proof of bounded stack; record a measured worst-case stack on the host in `docs/stack.md` (host figure, not an MCU claim). `sgs_sim`, pybind11, ROS, and the Vite shell are exempt.

RNG is xorshift32 in sim only. Seed 0 is remapped to 1. Uniform and Gaussian conversion and draw order are documented in `fixtures/README.md`. Same seed, same scenario, same build produce the same double trace within `1e-9` rad. WASM float traces use a separate tolerance (`1e-5` rad) and are not compared to C++ goldens at `1e-9`. Do not golden-compare `compute_ms` or frame time.

## Pinned stack

Pins are exact. CI does not float `latest`. FetchContent uses these commits because tags can move. Record each archive SHA-256 beside the declare on the first green configure (`cmake/pins.sha256`) and enforce `URL_HASH`. Checked 2026-11-10.

| Component | Pin | Source |
| --- | --- | --- |
| Language | C++17, no GNU extensions, no Eigen | Small `Quat` / `Vec3` / `Mat3` only |
| Core CI | `ubuntu-24.04`, `g++-13` selected explicitly, Python 3.12 | GitHub Actions. Log actual compiler/Python versions. Python 3.12 is supported through 2028-10-31, not the newest feature release. |
| Humble CI | `ros:humble-ros-base` **digest + linux/amd64**, pinned on the first green run. g++ 11 and Python 3.10 are whatever that digest ships (3.10 is EOL upstream as of 2026-10-01). | Humble supported until May 2027 ([REP-2000](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst)) |
| CMake | minimum 3.22; also confirmed to configure on 4.4.5 | https://github.com/Kitware/CMake/releases/tag/v4.4.5 |
| GoogleTest | v1.18.0, commit `063de7e9578f82b369302001269680b4b1553359` | https://github.com/google/googletest/releases/tag/v1.18.0 |
| pybind11 | v3.1.0, commit `97bf890db679505a14dfe547a5e77bb2bd05dc90` (peeled tag; CMake `Python_*`) | https://github.com/pybind/pybind11/releases/tag/v3.1.0 |
| nlohmann/json | v3.12.0, commit `55f93686c01528224f448c19128836e7df245f72` (MIT; NOTICE). Tests/scenarios only; not included by core headers. | https://github.com/nlohmann/json/releases/tag/v3.12.0 |
| pytest | 9.2.0, lock transitive deps and hashes in `python/requirements-dev.txt` | https://pypi.org/project/pytest/ |
| Node | 26.13.0 Active LTS (Lithium, bundles npm 11.20.0). Node 20 is EOL (ended 2026-04-30). | https://nodejs.org/dist/index.json |
| TypeScript / Vite / Vitest | 7.0.2 / 8.3.7 / 5.1.0. Commit the lockfile. Vitest CI is `vitest run`. Vite 8.3.7’s published advisory fix depends on Rolldown 1.2.17 — confirm that in the lockfile. | npm registry / https://github.com/vitejs/vite/releases/tag/v8.3.7 |
| Playwright | `@playwright/test` 1.65.0 (depends on `playwright` 1.65.0). Chromium only, installed with `--with-deps` at that exact revision. | https://registry.npmjs.org/@playwright/test/latest |
| emsdk | Manager tag 7.0.0, git commit `1693ce7359fc9694f198694f902a2cc061239b86`. Install and activate **7.0.0**, never `latest`. That tag maps to emscripten-releases build `6afbd83493d647051772d491124187d4e9a0cda9` (from `emscripten-releases-tags.json` on the tag). Record `emcc --version` on first green CI. | https://github.com/emscripten-core/emsdk/blob/7.0.0/emscripten-releases-tags.json |
| ROS apt | Full names: `ros-humble-rclcpp`, `ros-humble-sensor-msgs`, `ros-humble-geometry-msgs`, `ros-humble-visualization-msgs`, `ros-humble-rosidl-default-generators`, `ros-humble-rosidl-default-runtime`, `ros-humble-ament-cmake`. No version snapshot; say so. Jazzy is not built. | Humble package names |
| Actions | Pin `actions/checkout`, `setup-node`, `setup-python`, `upload-artifact`, `configure-pages`, `deploy-pages` to full commit SHAs when the workflows are first written; record them in `.github/workflow-pins.md`. | https://docs.github.com/en/actions/reference/security/secure-use |
| Plots | Canvas 2D in this repo. No chart library. uPlot stays an unused idea. | — |

## File layout

```text
LICENSE  NOTICE  README.md  CMakeLists.txt  cmake/pins.sha256
cpp/include/sgs/   cpp/src/core/   cpp/src/sim/   cpp/test/   cpp/tools/noheap_link.cpp
tools/check_no_heap.py
python/sgs/sgs_core.pyi  python/tests/  python/requirements-dev.txt  python/pytest.ini
ros2/sgs_gimbal/   (package.xml, CMakeLists.txt, gimbal_node.cpp, srv/GetStatus.srv, launch/gimbal.launch.py)
web/   (Vite + TS UI, wasm glue, playwright.config.ts)
fixtures/README.md
.github/workflows/{core,web,pages,ros2-humble}.yml  .github/workflow-pins.md
docs/demo.png  docs/stack.md
```

Namespace `sgs`. `SGS_BUILD_TESTS` default ON. `SGS_BUILD_PYTHON`, `SGS_BUILD_ROS`, and `SGS_BUILD_WASM` default **OFF**. Criterion 1 is a Python-free, ROS-free configure. Core objects never link `rclcpp`. Static core/sim libraries are built PIC so they can be linked into the optional pybind module. FetchContent of gtest / pybind / json is conditional on the options that need them.

## Interfaces

```text
struct Quat { double w, x, y, z; };  struct Vec3 { double x, y, z; };
struct Mat3 { double m[9]; };  // row-major, POD

struct ImuSample { double gyro_rad_s[3]; double accel_m_s2[3]; double mag_uT[3];
  uint32_t flags; };  // bit0 mag_valid, bit1 accel_valid
struct EncoderSample { double pitch_rad, yaw_rad; };
struct AttitudeSetpoint { Vec3 boresight_ned; };  // unit +x direction in world; zero vector => hold

struct PidGains { double kp, ki, kd; };
struct Gains { PidGains rate, boresight; double tau_d_s;
  double ekf_sigma_gyro, ekf_sigma_bias_rw, ekf_sigma_acc, ekf_sigma_mag;
  double comp_kp_acc, comp_kp_mag; };
struct Limits { double rate_rad_s, tau_max_nm, deadzone_nm, theta_warn_rad;
  double accel_gate_frac, mag_gate_frac; };

struct PidState { double integ, d_filt, meas_prev; bool has_meas; };
struct EstimatorState { Quat q; double gyro_bias[3]; double P[36]; /* comp bias is always 0 */ };
struct ControllerState {
  PidState rate_pitch, rate_yaw, att_pitch, att_yaw;
  EstimatorState comp, ekf;
  enum class EstimatorSelect : uint8_t { Complementary, Ekf } select;
  Vec3 last_boresight; bool has_setpoint; bool ready;
};

enum class Status : uint8_t { Ok, InvalidDt, InvalidInput, NotReady };
struct MotorCommand { double tau_pitch, tau_yaw; Status status; uint32_t flags; };
// flags: sat_pitch, sat_yaw, near_singularity, accel_gated, mag_missing, mag_weak
struct StepInput { ImuSample imu; EncoderSample enc; AttitudeSetpoint set; };
struct StepOutput { MotorCommand motor; Vec3 boresight_error_rad; Vec3 omega_sp;
  double pitch_rate_sp, yaw_rate_sp; };

void reset_state(ControllerState* state, const Quat& q_init);  // unit quat required; default identity
class Controller { // no heap; state is caller-owned, 8-byte aligned
  Status step(double dt, const Gains&, const Limits&, const StepInput&,
              ControllerState* state, StepOutput* out) const;
};
```

`step` uses stack locals and `*state` only. Null `state` or `out`, or aliased buffers, returns `InvalidInput` without dereferencing. `dt` out of range: `InvalidDt`, zero torques, initialised outputs, unchanged state. Any NaN or Inf in gyro, accel (when `accel_valid`), encoder, or a non-zero boresight with norm outside `[0.9, 1.1]` returns `InvalidInput` the same way. A zero boresight vector holds the last **accepted attitude setpoint**, not the last torque; on first use before any setpoint it holds the current estimated boresight. Missing mag sets `mag_missing`, skips both mag updates, and still runs accel tilt and gyro predict. `accel_valid` clear skips accel updates. World magnetic reference is a documented fake field `[25, 0, 40] µT` in NED (declination 0), not a WMM lookup. Changing `select`, gains, or limits mid-run must not kick the derivative (treat as `has_meas = false` on that change, or provide `reset_filters`).

`sgs_sim` types `PlantParams`, `PlantState`, `SensorModel`, `Disturbance`, and `run_scenario(seed, Scenario, Trace*)` are not declared in core headers. Trace storage may use `std::vector` in the sim `.cpp` only.

ROS node `sgs_gimbal_node` (only if `SGS_BUILD_ROS`): **converts NED/FRD to ENU/FLU at the adapter** (REP-103 / REP-145). Internal core stays NED. Publishes `/gimbal/imu` (`sensor_msgs/Imu`, best effort, depth 10), `/gimbal/joint_states` (`sensor_msgs/JointState`, best effort, depth 10), `/gimbal/camera_attitude` (`geometry_msgs/QuaternionStamped`, reliable, depth 10), and a small `/gimbal/markers` triad plus the needed static TF. Subscribes to `/gimbal/setpoint` (`geometry_msgs/Vector3Stamped` boresight in ENU, reliable, depth 10). Service `sgs_gimbal/GetStatus.srv` is a real generated interface. The node starts **paused**; `get_status` before `start` returns `NotReady` with reason `no_plant`. No lifecycle, no hardware plugin, no real IMU driver. Message quaternion order is `x,y,z,w`. `use_sim_time` is documented.

Python module `sgs_core` (optional) wraps `Controller::step` and `run_scenario`. Stubs in `sgs_core.pyi` with SPDX. Built into `build/python/`. `python/pytest.ini` sets `pythonpath`. Goldens are asserted in gtest; optional pytest checks one seeded trace against the same tolerances.

WASM: Emscripten ES-module glue plus `sgs.wasm`. Exports are `extern "C"`: `sgs_reset`, `sgs_set_gains`, `sgs_step_ms` (integer millisecond count at fixed 1 ms, **capped** per call, e.g. 50 ms, so the page cannot hang), and a preallocated float trace buffer (schema, stride, and capacity documented; views are reacquired if memory grows, or memory is fixed). `locateFile` / `import.meta.env.BASE_URL` must honour `/sim-gimbal-stabilizer/`. Vite `base` is that path. Native and WASM are **separate** build directories. The web build script compiles WASM with the pinned emsdk **before** Vite copies `web/public`. Use Node 26.13.0 for npm, not emsdk’s bundled Node.

## Edge cases

- `dt <= 0` or `dt > 0.002`, NaN/Inf gyro: status as above, no integration, zero torque.
- Zero boresight holds last accepted setpoint; first-use hold uses current estimate. Norm 0.5 is `InvalidInput`.
- `mag_valid` clear: yaw correction off; accel must not invent heading.
- Specific force `1.4 g`: accel update skipped, `accel_gated` set; predict still runs.
- Pitch encoder 85°: `near_singularity`, finite yaw torque. Plant clamp at 80° zeroes outward rate. Recovery back through 70° clears the flag (with hysteresis of 2°).
- Zero gains at 0.5 Hz do not meet the closed-loop isolation bound (ratio `≥ 0.90`).
- Base roll 0.5° at 8 Hz is not rejected (ratio in `[0.70, 1.30]`). The plot is labeled residual roll.
- Saturation: a large boresight step hits `±τ_max` and the integrator unwinds (back-calculation test with a stuck measurement).
- Dead-zone: `|τ| < 0.001` publishes 0.
- Same seed twice, same build: max abs sample difference `≤ 1e-9`.
- Mag-off EKF: yaw bias does not converge as if observed.

## Test plan

gtest in `cpp/test` links core and, for scenarios, sim. No ROS headers. CTest `TIMEOUT` is 2 s for unit cases and 30 s for scenario suites.

- Math: Hamilton multiply, normalize, shortest-path and `w == 0` tie, **+90° pitch basis vector** (`+x → (0,0,−1)` in the parent of the pitch joint), the LS map at `θ = 0` and `θ = 30°`, and the 85° authority flag.
- PID: P, I, D-on-measurement, filter, saturation, back-calculation unwind, dead-zone, first-sample derivative init.
- Complementary static tilt, and the biased-gyro equilibrium (`≈ 0.29°`) within `0.15°`.
- `kalman_update` 1-state hand values `K = x = P = 0.5`. EKF static-bench bias convergence with mag on. Mag-off unobservability. Finite-difference Jacobian check.
- `noheap_link` plus `check_no_heap.py`.
- Closed loop, seed 1, perfect sensors: 10° pitch and 10° yaw steps against the rise, overshoot, and settle bounds. Zero-gain twin does not track.
- Isolation at 0.5 Hz pitch and yaw: CL `≤ 0.35`, OL `≥ 0.90`. Isolation at 10 Hz: CL `≤ 0.35` (no OL lower bound).
- Residual-roll transmissibility in `[0.70, 1.30]`.
- Seed 2, noisy combined vibration and gusts: boresight-error RMS `≤ 1.5°` versus the held target.
- Determinism: two runs, `1e-9` rad. Invalid-input cases above.

Playwright (`web.yml`): build WASM, `npm ci`, `tsc --noEmit`, `vitest run`, `vite preview` with `base: '/sim-gimbal-stabilizer/'`, open that path, wait until the module initialises and a trace counter advances, move the boresight `k_p` slider and assert a **numerical** output changes, run the pitch step, request the `.wasm` URL and expect `content-type` to contain `application/wasm`, write `web/test-results/demo.png` and upload it as a CI artifact. Chromium only, `npx playwright install --with-deps chromium`. A committed `docs/demo.png` is produced locally and checked in once; CI does **not** auto-commit it. vitest covers UI helpers only (slider clamping, base-path join, locateFile). It does not reimplement control.

Workflows on push and pull_request:

- `core.yml`: `ubuntu-24.04`, g++-13, cmake, ctest, no-heap lint. Python/pytest only if that job explicitly enables `SGS_BUILD_PYTHON`. No ROS apt packages.
- `web.yml`: Node 26.13.0, pinned emsdk 7.0.0, WASM compile, `npm ci`, `tsc --noEmit`, vitest run, Playwright.
- `pages.yml`: on `main` only, after core and web succeed (`needs` the tested `web/dist` **artifact**, do not rebuild untested). `actions/deploy-pages` with required permissions. No PR deploy.
- `ros2-humble.yml`: `pull_request` types include `labeled` / `synchronize` / `opened`. Runs on push to `main` and on PRs labeled `ci-ros`. Colcon-build the wrapper against an installed core+sim (PIC, CMake package config). Smoke: start paused, `GetStatus` returns `no_plant`, start, await one `QuaternionStamped` with a bounded timeout and matching QoS. Setup/build up to 20 minutes; smoke itself capped at 3 minutes. A red Humble job does not waive a red core job.

## Numbered success criteria

1. `cmake -S . -B build -DSGS_BUILD_PYTHON=OFF -DSGS_BUILD_ROS=OFF -DSGS_BUILD_WASM=OFF && cmake --build build && ctest --test-dir build` passes on ubuntu-24.04 / g++-13 with no ROS and no Python module required.
2. Core TUs build with `-fno-exceptions -fno-rtti`. `check_no_heap.py` and `noheap_link` pass.
3. 10° pitch and 10° yaw boresight steps meet overshoot `≤ 25%`, rise `≤ 0.40 s`, and `±0.5°` on every sample from 1.5 s through 3.0 s with `default_gains()` and perfect sensors. The zero-gain twin does not track (stays within `0.2°` of the start).
4. At 0.5 Hz and 1° base amplitude, pitch and yaw closed-loop transmissibility are `≤ 0.35` and the zero-gain twin is `≥ 0.90`. At 10 Hz, closed-loop transmissibility is `≤ 0.35`. The README states that 10 Hz isolation is largely passive.
5. Seeded vibration plus 1-cosine gusts keeps truth boresight-error RMS `≤ 1.5°` versus the held target. A dedicated base-roll test has residual-roll transmissibility in `[0.70, 1.30]`. The README says a 2-axis gimbal does not reject roll.
6. Complementary static tilt converges under 1° in 3 s with zero bias. The biased-gyro case matches the predicted equilibrium within `0.15°`. The README states the filter does not estimate gyro bias.
7. EKF static bench (mag on) brings a 0.01 rad/s constant yaw bias inside 0.003 rad/s by 8 s. The 1-state update matches `K = x = P = 0.5` within `1e-12`. Mag-off does not silently observe yaw bias.
8. Two runs with the same seed and same build match within `1e-9` rad. Tests do not compare wall-clock times.
9. Playwright loads `/sim-gimbal-stabilizer/`, waits for WASM init, moves a gain and sees a numerical change, runs a step, checks the wasm MIME type, and uploads `demo.png`. The control code in that page is the WASM core, not a TypeScript PID or EKF.
10. The Humble job is optional, uses the pinned image digest once recorded, converts frames at the adapter, and is not required for criteria 1–2. Jazzy is not a v1 job.
11. `LICENSE` is byte-identical to `https://www.apache.org/licenses/LICENSE-2.0.txt`. `NOTICE` starts with `sim-gimbal-stabilizer, Copyright 2026 Arpan Bom`. Authored sources carry `SPDX-License-Identifier: Apache-2.0`.
12. README says portfolio demo, simulation only, camera boresight / shake rejection, Apache-2.0, torque plant rather than FOC/BLDC, simplified per-joint plant, and no production firmware, flight test, or certification claim.
13. The tree has no Quad-X mixer, altitude loop, quad plant, UART motor FSM, or Rust failsafe, and it does not vendor PX4, SimpleFOC, Fusion, TinyEKF, or imu_tools.
14. Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.
15. Default gains published in the README are the locked set above and are the gains that satisfy criteria 3 and 4. There is no v1 base-rate feed-forward.
16. Accel-only updates are documented as blind in yaw. Mag-off tests skip yaw correction. The 0.5 Hz isolation claim is the feedback proof; the 10 Hz claim is pitch and yaw only.
17. Positive pitch is tested as upward in FRD (`+x` toward `−z`). The inner loop is documented and tested as inertial-rate (gyro), not as a joint-rate encoder servo.

## Out of scope

FOC, SVPWM, MOSFET drivers, STM32 or RP2040 HALs in the default build, flashing, real IMUs, encoder protocols, 3-axis isolation claims, full-attitude control with two joints, vision tracking, target tracking, mechanical CAD, a certified Dryden or multi-body spatial plant, base IMU / base-rate feed-forward, Windows CI, Jazzy CI, copying PX4 / Arduino-FOC / Fusion / TinyEKF / imu_tools sources, accounts, and hosted services. This is a camera-stabilization teaching sim. README and UI copy say shake rejection / boresight hold only. No weapon, cueing, or fire-control language.

## Documentation and honesty

README order: (1) portfolio demo, simulation only, not flown, not a product the author shipped; (2) screenshot and what the two loops do (boresight + inertial rate); (3) build core tests, optional Python, optional WASM dev server, ROS marked optional; (4) frames, **positive pitch = up in FRD**, joint order, rates, locked gains, 75° authority flag; (5) plant equation, the teaching-approximation disclaimer, vibration lines, 1-cosine gusts, not Dryden and not FOC; (6) estimator honesty: gravity does not observe yaw, complementary filter does not estimate gyro bias, magnitude accel gate is incomplete, residual roll remains, 10 Hz isolation is largely passive; (7) Apache-2.0, `NOTICE`, idea credits, code written here; (8) author Arpan Bom.

Idea sources, not code to copy, are listed in `REFERENCES.md`. Keep that file in the planning set and copy a short credit list into the repo README at build time.

## License

Apache-2.0, not MIT.

- `LICENSE` is the verbatim file at `https://www.apache.org/licenses/LICENSE-2.0.txt`, fetched when the repo is created, not rewritten. Verify byte equality with the official text (including the appendix).
- `NOTICE` opens with `sim-gimbal-stabilizer, Copyright 2026 Arpan Bom`. List idea-credit projects and their licenses separately from **dependency** notices (GoogleTest BSD-3-Clause, pybind11 BSD-style, nlohmann/json MIT, and any Emscripten/runtime notices that ship in the Pages bundle). State that no third-party **controller** source is vendored, without claiming “no third-party code” if FetchContent or WASM glue is distributed.
- Authored C++, Python (including `.pyi`), TypeScript, CMake, launch, shell, and workflow files start with `SPDX-License-Identifier: Apache-2.0` in a valid comment, after any shebang or XML declaration. Lockfiles, generated stubs, and Emscripten glue are exempt and keep their upstream licenses. Never relabel third-party files with this project’s license.

## Build and CI

```text
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSGS_BUILD_TESTS=ON
cmake --build build -j && ctest --test-dir build --output-on-failure
python3 tools/check_no_heap.py
# optional:
cmake -S . -B build -DSGS_BUILD_PYTHON=ON && cmake --build build -j
PYTHONPATH=build/python python3 -m pytest -q python/tests
```

WASM is a preset that sources the pinned emsdk, points `CMAKE_TOOLCHAIN_FILE` at that SDK’s `Emscripten.cmake`, builds `sgs_core` plus `sgs_sim` (Python/ROS off) into a separate directory, and installs glue + `sgs.wasm` into `web/public/` before Vite runs. Web CI: WASM compile, then `npm ci && npx tsc --noEmit && npx vitest run && npm run build` and Playwright against `vite preview`. Pages deploys the **tested** `dist` artifact only from `main` after core and web are green. Humble installs the core+sim to a prefix, then colcon-builds `ros2/sgs_gimbal` with `SGS_BUILD_ROS=ON`.

## Risks

- High-frequency isolation is mostly inertia. The 0.5 Hz OL/CL pair is the check that feedback does something. Do not “fix” a soft plant by stiffening the spring just to flunk open-loop at 10 Hz.
- A later two-axis coupled plant can miss one-axis feasibility numbers. If that happens, retune or adjust bounds in the same commit and say so; do not silently weaken tests.
- `operator new` hooks in `noheap_link` must not pull in sim, gtest, or pybind. The lint is incomplete by itself.
- WASM MIME, `BASE_URL`, glue `locateFile`, and memory-growth view invalidation fail in ways a root `vite` dev URL will not. Playwright hits the production base and waits for init.
- A reader can confuse this with `cpp-flight-controller` or a FOC driver, or think the gyro loop is a joint-rate servo. The README lead states the difference. Review rejects a mixer, altitude loop, SVPWM file, or “joint-rate from gyro_z/cosθ” comment.
- Complementary-filter marketing often implies bias estimation. Tests and the README say this one does not.
- The Humble image’s Python 3.10 is EOL. It stays inside that optional job only. ROS messages are ENU at the boundary; mixing that with internal NED would be a silent bug.
- FetchContent tags move. Commits and archive SHA-256 are required. The emsdk **manager** SHA and the emscripten-releases **build** SHA are different namespaces; both are recorded, and `latest` is never activated.
- PX4’s gimbal docs are easy to over-claim. Credit them as a mount router plus general attitude ideas, not as the source of this stabilizer.

## Verification already done (do not re-litigate)

- Existing applycoding repos were read; this design does not duplicate them.
- Eight primary references are MIT, BSD-3-Clause, and/or Apache-2.0 (see REFERENCES.md). GPL/LGPL and STorM32/Basecam are flagged idea-only or excluded.
- Pins checked 2026-11-10: pytest 9.2.0, pybind11 3.1.0, TypeScript 7.0.2, Vite 8.3.7, Vitest 5.1.0, Playwright 1.65.0, GoogleTest v1.18.0 SHA, nlohmann/json v3.12.0 peeled SHA, Node v26.13.0 Lithium, CMake 4.4.5, emsdk 7.0.0 manager SHA and releases map `6afbd83…`, Apache license URL 200, PX4 gimbal docs 200, job post 200, arXiv 1711.02508 200. The Valenti paper HTML at mdpi.com returned 403 to this environment; the DOI `10.3390/s150819302` redirects to that page and is the citation.
- Independent Python one-axis plant/PID (semi-implicit Euler, locked gains, specified PID law): Kalman hand example exact; discrete closed-loop spectral radius `0.9998` (all poles inside the unit circle); 10° step overshoot `2.0%`, rise `0.236 s`, max 1.5–3 s error `0.19°`; 0.5 Hz OL `1.007` / CL `0.204`; 10 Hz OL `0.212` / CL `0.157`; combined noisy RMS `0.047°`. Linear frictionless `|H(j 2π 10)| ≈ 0.041`, `f_n ≈ 1.84 Hz`. The draft’s 10 Hz open-loop `≥ 0.70` guard is not used.
