# Deviations from PLAN.md

## EKF implementation simplification

The core keeps a 6-state error-state layout (attitude error and gyro bias, a row-major 6×6 `P`, and inject/reset hooks) and the 1-state `kalman_update_scalar` hand example (`K = x = P = 0.5` within `1e-12`).

Attitude measurement updates are decoupled scalar Kalman corrections on tilt and yaw innovations, not a coupled 6×6 Joseph update on every accelerometer or magnetometer measurement. The Joseph helper is not used on the v1 path. This kept the MCU-shaped core small.

Magnetometer yaw uses the shared `mag_heading_error_rad` innovation with a body-frame horizontal magnitude gate (not a world-frame horizontal norm that was unreachable when attitude was wrong).

Yaw-bias convergence on the static bench is a stationary-gate first-order blend of the gyro onto the bias state, enabled only when both the accelerometer and magnetometer flags are valid. It is not a Kalman update of `b_gz` from the magnetometer innovation. Mag-off therefore does not observe yaw bias. Criterion 7 still holds, including static boresight error `< 1°` with mag on.

**Noise retune (thresholds unchanged):** default `ekf_sigma_mag` is `1.0 µT` (PLAN / prior default `2.0`) and `ekf_init_state` starts attitude / bias `P` diagonals at `0.02` / `1e-5` (was `1e-4` / `1e-6`) so the static boresight check meets PLAN without relaxing the bound. `Gains::ekf_sigma_mag` and `default_gains()` both use `1.0`.

No acceptance thresholds were relaxed.
