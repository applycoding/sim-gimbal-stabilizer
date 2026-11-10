# Build tasks — sim-gimbal-stabilizer

Execute in order. Follow PLAN.md. Deviate only if a pin or test proves a step broken, and record why. License is **Apache-2.0**, not MIT. Commit author and committer are Arpan Bom <applycoding@gmail.com>; no co-author trailers.

Each task has acceptance checks. A task is done only when those checks pass.

## 1. Repository skeleton and license

- Create the tree in PLAN.md (empty dirs may hold `.gitkeep`).
- Fetch `https://www.apache.org/licenses/LICENSE-2.0.txt` into `LICENSE` and confirm byte equality (including the appendix).
- Write `NOTICE` starting with `sim-gimbal-stabilizer, Copyright 2026 Arpan Bom`, then idea-credit licenses and a separate dependency-notice section (filled as FetchContent and WASM glue land).
- README stub: portfolio demo, simulation only, Apache-2.0, author Arpan Bom, camera boresight / shake rejection, no production, flight-test, FOC, or real-hardware claims.
- Root `CMakeLists.txt` (C++17, CMake >= 3.22, options `SGS_BUILD_TESTS` default ON, `SGS_BUILD_PYTHON` / `SGS_BUILD_ROS` / `SGS_BUILD_WASM` default OFF).
- Add `SPDX-License-Identifier: Apache-2.0` on every authored file created from here on (after shebangs / XML declarations).

**Accept:** `cmp LICENSE` against a fresh curl of the official text exits 0. NOTICE and README name Apache-2.0 and Arpan Bom. `cmake -S . -B build` configures with Python, ROS, and WASM off.

## 2. FetchContent pins and third-party notices

- Add GoogleTest (and, only when the matching option is on, pybind11 and nlohmann/json) via FetchContent at the **commit SHAs** in PLAN.md. Record archive SHA-256 and set `URL_HASH`.
- Append BSD/MIT notices for those dependencies to `NOTICE`. Do not relicense them. Do not add SPDX Apache headers to their sources.

**Accept:** Configure+build with tests on fetches the pinned commits (not a moved tag). A Python-off configure does not require pybind11. NOTICE lists each pulled dependency and its license.

## 3. Math types, frames, and allocation

- Implement `Quat`, `Vec3`, `Mat3`, Hamilton product, normalize, shortest-path sign, `w == 0` tie-break, and `Exp` for `q ⊗ Exp(ω dt)`.
- Implement the yaw-then-pitch product and the least-squares inertial-rate map `θ̇ = ω_y`, `ψ̇ = −sin θ · ω_x + cos θ · ω_z`, plus the torque map in PLAN.md.
- Positive pitch: +90° takes parent `+x` to `(0, 0, −1)` (upward in FRD).

**Accept:** gtests for the +90° basis vector, the LS map at `θ = 0` and `θ = 30°`, sign invariance, and a documented `w == 0` tie. No `1/cos θ` in the production map.

## 4. PID and defaults

- Implement the exact discrete PID law in PLAN.md (D on measurement, back-calculation, first-sample `d_raw = 0`, inner dead-zone `0.001 N·m`).
- `default_gains()` and `default_limits()` match the locked numbers in PLAN.md.

**Accept:** gtests for P, I, D-on-measurement, filter, saturation, back-calculation unwind with a stuck measurement, dead-zone of `0.001`, and first-sample init. Units are documented.

## 5. Complementary filter

- Mahony-style proportional correction, no bias state. Defaults `comp_kp_acc = 2.0 /s`, `comp_kp_mag = 0.5 /s`. Specific-force model as in PLAN.md (`level rest = [0, 0, −g]` in FRD). Mag yaw correction skipped when mag is invalid or weak.

**Accept:** 20° static tilt, zero bias, `|error| < 1°` by 3 s. Constant gyro bias `0.01 rad/s` settles near `0.29°` within `0.15°` and is not required to go to zero. Accel-only (mag off) does not invent heading.

## 6. Error-state EKF

- 6-state ESKF as in PLAN.md: `[δθ, b_g]`, Joseph-form (or equivalent) update, inject and reset, `dt`-scaled process noise, accel and mag gates, `mag_weak` path.
- Shared 1-state `kalman_update` used by the 6-state filter.

**Accept:** hand values `K = x = P = 0.5` within `1e-12`. Static bench with mag on: `0.01 rad/s` yaw bias inside `0.003 rad/s` by 8 s, boresight error `< 1°`. Mag-off (or vertical-only field) does not silently observe yaw bias. A finite-difference Jacobian check passes.

## 7. Controller::step, reset, and status

- `reset_state` requires a unit quaternion (default identity). `step` matches the contracts in PLAN.md: dt window `(0, 0.002]`, zero-torque + unchanged state on `InvalidDt` / `InvalidInput`, hold last accepted boresight, coincident **flags**, selected estimator’s bias in the rate loop, `near_singularity` when `|θ| > 75°` with 2° hysteresis, frozen yaw integrator in that region, no v1 feed-forward.

**Accept:** gtests for invalid dt/NaN/null, hold semantics on first use and later, estimator switch that does not kick the derivative, 85° encoder flag with finite torque, and flag combinations (gated + saturated).

## 8. Sim plant, sensors, and disturbances

- Per-joint teaching plant, frozen numbers, semi-implicit Euler, hard-stop at 80° with outward rate zeroed, command dead-zone applied as specified.
- Sensor model (bias, white noise, optional walk, 12-bit encoder wrap).
- Scripted base motion, vibration lines, 1-cosine gusts, xorshift32 (seed 0 → 1), documented Gaussian conversion in `fixtures/README.md`.

**Accept:** a host runner writes a CSV trace. Open-loop 10 Hz 1° transmissibility is clearly isolated (order 0.2 or less with Coulomb), not forced near 1. Hard-stop does not increase `|ω_rel|` outward. README calls the plant a simplified per-joint model and not FOC.

## 9. Closed-loop scenario tests

- 10° pitch and 10° yaw boresight steps vs criterion 3, including the zero-gain twin.
- 0.5 Hz and 10 Hz isolation vs criterion 4, with warmup/window and sin/cos+DC fit as specified.
- Residual-roll test vs criterion 5.
- Seeded combined vibration + gust vs the 1.5° boresight-error RMS bound.
- Same-seed determinism at `1e-9`.

**Accept:** named gtests pass on ubuntu with the locked default gains. Metrics use truth boresight error versus the held target. CTest timeouts are set.

## 10. No-heap enforcement

- Token-aware `tools/check_no_heap.py` with a documented allowlist.
- `noheap_link` as a plain executable, core objects only, abort/counter hooks for `new`/`malloc` family, scenario covering both estimators, gating, saturation, and 85°.
- Host stack measurement recorded in `docs/stack.md` as a host figure, not an MCU rating.

**Accept:** criterion 2 holds. Accidental `std::vector` in a core `.cpp` fails the lint. The hook binary does not link gtest, sim traces, or pybind.

## 11. Optional pybind11 and pytest

- `SGS_BUILD_PYTHON=ON` builds `sgs_core` into `build/python/`. `pytest.ini` pythonpath set. Authored `sgs_core.pyi` with SPDX. PIC static libs.

**Accept:** `PYTHONPATH=build/python python3 -m pytest -q python/tests` passes on Python 3.12. Default configure still has Python off, and criterion 1 does not require this task.

## 12. WASM build and Vite app

- Pin emsdk 7.0.0 (manager SHA and install target in PLAN.md, never `latest`). Separate WASM build dir, `emcc --version` logged.
- ES-module glue, `extern "C"` exports, capped `sgs_step_ms`, preallocated trace buffer, `locateFile` / `BASE_URL` `/sim-gimbal-stabilizer/`.
- Canvas 2D: live boresight error, step button, frequency-sweep button, gain and disturbance sliders, residual-roll trace labeled as such. No TypeScript PID/EKF.
- Web build compiles WASM before copying into `web/public`.

**Accept:** `npm ci && npx tsc --noEmit && npx vitest run && npm run build` succeeds on Node 26.13.0 after the WASM compile. A local preview under the base path initialises the module and runs a step.

## 13. Playwright smoke and screenshot

- Chromium only, `npx playwright install --with-deps chromium` at Playwright 1.65.0. Preview the **built** site at the Pages base path. Wait for init and a advancing trace. Move a gain and assert a numerical change. Run the pitch step. Check wasm MIME. Write `web/test-results/demo.png` and upload it as a CI artifact. Copy a local `docs/demo.png` into the tree once; do not auto-commit from CI.

**Accept:** Playwright is green locally and the screenshot exists. Criterion 9 holds.

## 14. Optional ROS 2 Humble wrapper

- `ros2/sgs_gimbal` with `package.xml` (Apache-2.0), `GetStatus.srv`, `rosidl` generation, NED/FRD → ENU/FLU at the adapter, QoS, paused start, stamped I/O, `gimbal.launch.py`. Installed core+sim found via CMake package config. PIC.
- Smoke: `no_plant` before start, then one attitude message after start, bounded waits, matching QoS.

**Accept:** Colcon build on Humble succeeds. Smoke passes. Core tests still run with `SGS_BUILD_ROS=OFF` and no ROS installed.

## 15. CI and GitHub Pages

- `core.yml`, `web.yml`, `pages.yml` (deploy tested `dist` artifact from `main` only after core+web), `ros2-humble.yml` (label `ci-ros`, 20 min setup/build, 3 min smoke).
- Pin Actions to full SHAs in `.github/workflow-pins.md`. Pin `ros:humble-ros-base` digest + platform on the first green run. Record emsdk `emcc --version`.

**Accept:** Workflows exist and match PLAN.md. A first CI on the branch shows core and web green. Pages base path is used in the deploy artifact.

## 16. README, credits, and honesty pass

- Finish README in the order given in PLAN.md. Credit idea sources and their licenses from REFERENCES.md. Flag GPL/LGPL/STorM32/Basecam as idea-only or excluded. Document frames (positive pitch = up in FRD), inertial-rate loop, boresight objective, plant approximation, 0.5 Hz vs 10 Hz isolation honesty, estimator limits, quick start, and no SaaS.
- Footer and `package.xml` say Apache-2.0. Screenshot in README once `docs/demo.png` exists.

**Accept:** Criteria 11–13, 15–17 can be checked off by reading the tree. No MIT license artifacts remain for *this* repo. No FOC, 3-axis isolation, or production-firmware claims.

## 17. Git history hygiene (when the public repo is created)

- Every commit: `GIT_AUTHOR_NAME="Arpan Bom"`, `GIT_AUTHOR_EMAIL="applycoding@gmail.com"`, and the same for committer. No co-author trailers. Do not rewrite already-pushed history without explicit approval.

**Accept:** `git log --format='%an <%ae> %cn <%ce> %s %b'` shows only Arpan Bom as author and committer and no co-author trailers.

---

Do not start a follow-on 3-axis plant, FOC driver, base IMU, Jazzy port, or Idea Hub product in this repository.
