# References

Permissively licensed GitHub projects used as **references only**. Do not copy source. Credit any conceptually derived idea in the README. Licenses were checked on 2026-11-10 with `gh api repos/OWNER/REPO --jq .license.spdx_id`, and with the raw LICENSE, LICENSE.md, or package.xml when GitHub reported `NOASSERTION`.

**Outbound license for this demo repo is Apache-2.0**, not MIT. A LICENSE file with the official Apache-2.0 text, a NOTICE file crediting Arpan Bom, and SPDX-License-Identifier: Apache-2.0 headers are required at build time.

This demo must not duplicate existing applycoding repos:

| Repo | What it already is | How this demo differs |
| --- | --- | --- |
| [applycoding/cpp-flight-controller](https://github.com/applycoding/cpp-flight-controller) | C++20 quadrotor flight-control library: cascaded vehicle attitude/rate PID, Quad-X mixer, complementary filter, error-state EKF, altitude KF, rigid-body quad | Camera gimbal on a moving base. No mixer, no altitude loop, no quad plant. Math is reimplemented from first principles, not copied. |
| [applycoding/embedded-motor-controller](https://github.com/applycoding/embedded-motor-controller) | C++17 single-axis DC **speed** loop with UART protocol, faults, watchdog, STM32/RP2040-style HAL stubs | Multi-axis inertial boresight hold with a torque plant. No UART FSM, fault state machine, or watchdog. |
| [applycoding/rust-drone-failsafe](https://github.com/applycoding/rust-drone-failsafe) | Rust `no_std` failsafe supervisor, WASM, GitHub Pages | Different problem (failsafe logic). WASM-and-Pages layout is a process idea only. |

## Selected references (8)

### 1. PX4/PX4-Autopilot

- URL: https://github.com/PX4/PX4-Autopilot
- License: **BSD-3-Clause** (`gh api` SPDX: BSD-3-Clause; root LICENSE is the BSD 3-Clause text, copyright 2012–2025 PX4 Development Team)
- LICENSE: https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE
- Docs (checked 2026-11-10, HTTP 200): https://docs.px4.io/main/en/advanced/gimbal_control
- Gimbal module tree: https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/gimbal
- Borrow: cascaded attitude/rate thinking from the broader PX4 GNC stack, and the idea of a world-frame “hold this pointing” setpoint for a mount. **Honesty:** the `src/modules/gimbal` driver is mainly a mount **router** (RC / MAVLink v2 in, PWM or MAVLink out; fixed-attitude mode with `MNT_DO_STAB`). It is not a full onboard brushless stabilizer. Do not credit PX4 as the source of this repo’s PID/EKF stabilizer or copy module code.

### 2. simplefoc/Arduino-FOC

- URL: https://github.com/simplefoc/Arduino-FOC
- License: **MIT** (`gh api` SPDX: MIT)
- Docs: https://docs.simplefoc.com
- Latest release seen 2026-11-10: v2.4.0
- Borrow: cascaded angle / velocity / torque loops, output limits, and anti-windup as *control-structure* ideas for a small MCU-style loop. **We do not implement FOC, SVPWM, or a BLDC driver.** Do not copy SimpleFOC sources, examples, or motor-driver code. The README must say this plant is a torque command into a second-order joint, not FOC.

### 3. xioTechnologies/Fusion

- URL: https://github.com/xioTechnologies/Fusion
- License: **MIT** (LICENSE.md starts “The MIT License (MIT)”, copyright 2021 x-io Technologies). `gh api` SPDX was empty; use the file.
- LICENSE: https://github.com/xioTechnologies/Fusion/blob/main/LICENSE.md
- Borrow: AHRS layout ideas — gyro integration, accelerometer and magnetometer corrections, optional gyro-bias handling in a small C library aimed at embedded use. We write our own complementary filter (no bias state) and our own ESKF. Do not copy Fusion headers or algorithms verbatim.

### 4. Mayitzin/ahrs

- URL: https://github.com/Mayitzin/ahrs
- License: **MIT** (`gh api` SPDX: MIT; LICENSE file checked)
- LICENSE: https://github.com/Mayitzin/ahrs/blob/master/LICENSE
- Borrow: textbook-style descriptions of complementary, Kalman, and related attitude filters, useful for explaining estimator trade-offs in the README. Implement our own code; do not lift the Python package.

### 5. simondlevy/TinyEKF

- URL: https://github.com/simondlevy/TinyEKF
- License: **MIT** (`LICENSE.md`; GitHub SPDX `NOASSERTION` — use the file, copyright Simon D. Levy)
- LICENSE: https://github.com/simondlevy/TinyEKF/blob/master/LICENSE.md
- Borrow: the idea of a tiny, header-sized, **static-allocation** EKF that can live on an MCU (no `malloc`). Write our own 6-state error-state filter and our own `kalman_update`. Do not copy `tinyekf.h` or the examples.

### 6. ros-controls/control_toolbox

- URL: https://github.com/ros-controls/control_toolbox
- License: **Apache-2.0** (`gh api` SPDX: Apache-2.0; LICENSE file is the Apache 2.0 text)
- LICENSE: https://github.com/ros-controls/control_toolbox/blob/master/LICENSE
- Borrow: PID-with-limits and anti-windup as an API shape (gains, command bounds, integral freeze/back-calculation as a documented choice). Reimplement the discrete law in PLAN.md. Do not vendor the package.

### 7. CCNYRoboticsLab/imu_tools (`imu_complementary_filter` only)

- URL: https://github.com/CCNYRoboticsLab/imu_tools
- License: **mixed**. The repository LICENSE file states that `imu_filter_madgwick` is **GPL-3.0-or-later** and that the rest is **3-clause BSD**. The `imu_complementary_filter/package.xml` (checked 2026-11-10) declares **BSD-3-Clause**. GitHub SPDX is `NOASSERTION`.
- LICENSE summary: https://github.com/CCNYRoboticsLab/imu_tools/blob/master/LICENSE
- Paper: Valenti, Terzi, and Bona, “Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs,” *Sensors* 15(8), 2015. DOI: https://doi.org/10.3390/s150819302 (the mdpi.com HTML returned 403 to this environment; cite the DOI).
- Borrow: the *idea* of a quaternion complementary / explicit complementary correction from gyro, accelerometer, and optional magnetometer, and the Valenti paper’s problem statement. Implement a small Mahony-style proportional law as specified in PLAN.md. **Do not use, read-to-implement, or translate `imu_filter_madgwick` (GPL-3.0-or-later).** Do not copy the BSD complementary-filter sources either; they are an idea reference only. Do not reuse figures or prose from the paper.

### 8. cra-ros-pkg/robot_localization

- URL: https://github.com/cra-ros-pkg/robot_localization
- License: **BSD-3-Clause** (LICENSE file: Charles River Analytics, 2014–2016; GitHub SPDX `NOASSERTION` — use the file)
- LICENSE: https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/LICENSE (path may vary by branch; the LICENSE file at the repo root was fetched 2026-11-10)
- Borrow: the idea of an EKF that fuses IMU (and other sensors) with documented frames and covariances. Far too large to vendor. Our ESKF is a tiny attitude+bias filter, written here.

## Papers (idea-only, not code)

- Joan Solà, “Quaternion kinematics for the error-state Kalman filter,” arXiv:1711.02508, https://arxiv.org/abs/1711.02508 (HTTP 200 on 2026-11-10). Use as a mathematical reference for error-state injection/reset. Do not copy code listings or figures into the tree.

## Looked at, not used as primary references

| Project | License | Why not primary |
| --- | --- | --- |
| [leeoniya/uPlot](https://github.com/leeoniya/uPlot) | MIT | Good small time-series plot library. v1 uses Canvas 2D in this repo; no runtime dependency. Optional later UX idea only. |
| [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) | Apache-2.0 | Useful as a later ROS 2 control-framework reading; v1’s optional wrapper is a thin `rclcpp` node, not a `ros2_control` hardware interface. |
| [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) | GPL-3.0 | **Idea-only.** Incompatible with Apache-2.0 if code were derived. Do not read sources to implement the controller. |
| [betaflight/betaflight](https://github.com/betaflight/betaflight), [iNavFlight/inav](https://github.com/iNavFlight/inav) | GPL-3.0 | Same rule. |
| [morgil/madgwick_py](https://github.com/morgil/madgwick_py) | LGPL-3.0 | **Idea-only.** Do not copy or translate. |
| [CCNYRoboticsLab/imu_tools](https://github.com/CCNYRoboticsLab/imu_tools) `imu_filter_madgwick` | GPL-3.0-or-later | **Idea-only.** The complementary-filter package in the same repo is the permitted idea reference (BSD-3-Clause). |
| [olliw42/storm32bgc](https://github.com/olliw42/storm32bgc) | Hardware TAPR OHL; firmware/GUI license is pointed at an external page and is **not** a clean in-tree MIT/BSD/Apache grant | **Not a primary reference. Do not copy firmware, GUI, or protocols.** Historical product context only. |
| Basecam / SimpleBGC / AlexMos | Proprietary | Do not copy firmware, protocols, or vendor tools. A public product page is not a source license. |

## Credit rule

README must list these projects, their licenses, and a one-line “what we took as an idea”. Any file that is conceptually derived gets a short comment or docs note pointing at the source paper or repo. No vendored third-party controller source.

## Compatibility with our Apache-2.0 outbound license

We do not vendor third-party controller source. Even so, every primary reference was checked so that later attribution or a short derived comment would stay legal under Apache-2.0.

| Inbound license | Compatible with Apache-2.0 outbound? | Our use |
| --- | --- | --- |
| Apache-2.0 | Yes | control_toolbox, ros2_control (backup) |
| MIT | Yes, keep copyright and permission notice | Arduino-FOC, Fusion, ahrs, TinyEKF, uPlot |
| BSD-3-Clause | Yes, keep copyright, conditions, and disclaimer | PX4-Autopilot, imu_complementary_filter, robot_localization, GoogleTest (dependency) |
| LGPL-3.0 | **No** for copied or derived source | **Idea-only.** morgil/madgwick_py |
| GPL-3.0 / GPL-3.0-or-later | **No** for copied or derived source | **Idea-only.** ArduPilot, Betaflight, iNav, imu_filter_madgwick |
| TAPR OHL hardware + unclear firmware terms | Not a clean firmware grant for an Apache-2.0 software repo | STorM32 excluded as a code reference |
| Proprietary | No | Basecam / SimpleBGC / AlexMos |

All eight primary references are MIT, BSD-3-Clause, and/or Apache-2.0. No primary reference is GPL or LGPL.
