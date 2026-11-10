# Deviations from PLAN.md

## EKF implementation simplification

The core keeps a 6-state error-state layout (attitude error and gyro bias, a row-major 6×6 `P`, and inject/reset hooks) and the 1-state `kalman_update_scalar` hand example (`K = x = P = 0.5` within `1e-12`).

Attitude measurement updates are decoupled scalar Kalman corrections on tilt and yaw innovations, not a coupled 6×6 Joseph update on every accelerometer or magnetometer measurement. The Joseph helper is not used on the v1 path. This kept the MCU-shaped core small.

Yaw-bias convergence on the static bench is a stationary-gate first-order blend of the gyro onto the bias state, enabled only when both the accelerometer and magnetometer flags are valid. It is not a Kalman update of `b_gz` from the magnetometer innovation. Mag-off therefore does not observe yaw bias. The numerical gates in criterion 7 still hold (bias within `0.003 rad/s` by 8 s; mag-off bias unchanged). This is not a full error-state EKF bias observer.

No acceptance thresholds were relaxed.
