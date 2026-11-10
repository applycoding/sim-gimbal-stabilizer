// SPDX-License-Identifier: Apache-2.0
#include "sgs/sim/sensor.hpp"

#include "sgs/complementary.hpp"
#include "sgs/math.hpp"

#include <cmath>

namespace sgs {
namespace sim {

void sample_sensors(double dt, const PlantState& plant, const Quat& q_world_from_camera,
                    const SensorModel& model, bool perfect, XorShift32* rng, ImuSample* imu,
                    EncoderSample* enc) {
  (void)dt;
  const double theta = plant.theta_rel;
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  const Vec3 omega_base_cam = quat_rotate(
      quat_conj(q_world_from_camera),
      Vec3{plant.base_omega_roll, plant.base_omega_pitch, plant.base_omega_yaw});
  const Vec3 omega_joint{-s * plant.omega_psi, plant.omega_theta, c * plant.omega_psi};
  Vec3 omega = vec3_add(omega_base_cam, omega_joint);

  const Quat qc = quat_conj(q_world_from_camera);
  // Specific force f = R^T (a_world - g_ned); a level FRD camera reports [0, 0, -g].
  Vec3 accel = quat_rotate(qc, Vec3{0.0, 0.0, -kG});

  if (!perfect) {
    omega.x += model.gyro_bias[0] + model.gyro_noise_sigma * rng->gaussian();
    omega.y += model.gyro_bias[1] + model.gyro_noise_sigma * rng->gaussian();
    omega.z += model.gyro_bias[2] + model.gyro_noise_sigma * rng->gaussian();
    accel.x += model.accel_bias[0] + model.accel_noise_sigma * rng->gaussian();
    accel.y += model.accel_bias[1] + model.accel_noise_sigma * rng->gaussian();
    accel.z += model.accel_bias[2] + model.accel_noise_sigma * rng->gaussian();
  }

  imu->gyro_rad_s[0] = omega.x;
  imu->gyro_rad_s[1] = omega.y;
  imu->gyro_rad_s[2] = omega.z;
  imu->accel_m_s2[0] = accel.x;
  imu->accel_m_s2[1] = accel.y;
  imu->accel_m_s2[2] = accel.z;
  imu->flags = kFlagAccelValid;

  const Vec3 m_ned{complementary::kMagRefNed[0], complementary::kMagRefNed[1],
                   complementary::kMagRefNed[2]};
  Vec3 mag = quat_rotate(qc, m_ned);
  if (!perfect && model.mag_enabled) {
    mag.x += model.mag_noise_sigma * rng->gaussian();
    mag.y += model.mag_noise_sigma * rng->gaussian();
    mag.z += model.mag_noise_sigma * rng->gaussian();
  }
  imu->mag_uT[0] = mag.x;
  imu->mag_uT[1] = mag.y;
  imu->mag_uT[2] = mag.z;
  if (model.mag_enabled) {
    imu->flags |= kFlagMagValid;
  }

  const double scale = 4096.0 / (2.0 * kPi);
  double pitch = plant.theta_rel;
  double yaw = plant.psi_rel;
  if (!perfect) {
    pitch += (rng->uniform01() - 0.5) / scale;
    yaw += (rng->uniform01() - 0.5) / scale;
  }
  enc->pitch_rad = pitch;
  enc->yaw_rad = yaw;
}

}  // namespace sim
}  // namespace sgs
