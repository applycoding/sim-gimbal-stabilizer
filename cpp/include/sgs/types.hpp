// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace sgs {

struct Quat {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3 {
  double m[9]{};  // row-major
};

struct ImuSample {
  double gyro_rad_s[3]{};
  double accel_m_s2[3]{};
  double mag_uT[3]{};
  uint32_t flags{0};
};

constexpr uint32_t kFlagMagValid = 1u << 0;
constexpr uint32_t kFlagAccelValid = 1u << 1;

struct EncoderSample {
  double pitch_rad{0.0};
  double yaw_rad{0.0};
};

struct AttitudeSetpoint {
  Vec3 boresight_ned{};
};

struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

struct Gains {
  PidGains rate{};
  PidGains boresight{};
  double tau_d_s{0.005};
  double ekf_sigma_gyro{0.01};
  double ekf_sigma_bias_rw{1e-4};
  double ekf_sigma_acc{0.3};
  double ekf_sigma_mag{2.0};
  double comp_kp_acc{2.0};
  double comp_kp_mag{0.5};
};

struct Limits {
  double rate_rad_s{2.0};
  double tau_max_nm{0.15};
  double deadzone_nm{0.001};
  double theta_warn_rad{75.0 * 3.14159265358979323846 / 180.0};
  double accel_gate_frac{0.15};
  double mag_gate_frac{0.30};
};

struct PidState {
  double integ{0.0};
  double d_filt{0.0};
  double meas_prev{0.0};
  bool has_meas{false};
};

struct EstimatorState {
  Quat q{1.0, 0.0, 0.0, 0.0};
  double gyro_bias[3]{};
  double P[36]{};
  uint16_t static_gyro_count{0};
};

struct ControllerState {
  PidState rate_pitch{};
  PidState rate_yaw{};
  PidState att_pitch{};
  PidState att_yaw{};
  EstimatorState comp{};
  EstimatorState ekf{};
  enum class EstimatorSelect : uint8_t { Complementary, Ekf } select{
      EstimatorSelect::Ekf};
  Vec3 last_boresight{1.0, 0.0, 0.0};
  bool has_setpoint{false};
  bool ready{false};
  bool near_singularity_latched{false};
};

enum class Status : uint8_t { Ok, InvalidDt, InvalidInput, NotReady };

struct MotorCommand {
  double tau_pitch{0.0};
  double tau_yaw{0.0};
  Status status{Status::NotReady};
  uint32_t flags{0};
};

constexpr uint32_t kFlagSatPitch = 1u << 0;
constexpr uint32_t kFlagSatYaw = 1u << 1;
constexpr uint32_t kFlagNearSingularity = 1u << 2;
constexpr uint32_t kFlagAccelGated = 1u << 3;
constexpr uint32_t kFlagMagMissing = 1u << 4;
constexpr uint32_t kFlagMagWeak = 1u << 5;

struct StepInput {
  ImuSample imu{};
  EncoderSample enc{};
  AttitudeSetpoint set{};
};

struct StepOutput {
  MotorCommand motor{};
  Vec3 boresight_error_rad{};
  Vec3 omega_sp{};
  double pitch_rate_sp{0.0};
  double yaw_rate_sp{0.0};
};

Gains default_gains();
Limits default_limits();

class Controller {
 public:
  Status step(double dt, const Gains& gains, const Limits& limits,
              const StepInput& in, ControllerState* state, StepOutput* out) const;
};

void reset_state(ControllerState* state, const Quat& q_init);

}  // namespace sgs
