// SPDX-License-Identifier: Apache-2.0
#include "sgs/math.hpp"

#include <cmath>

namespace sgs {

Vec3 vec3_add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 vec3_sub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 vec3_scale(const Vec3& v, double s) {
  return {v.x * s, v.y * s, v.z * s};
}

double vec3_dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 vec3_cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double vec3_norm(const Vec3& v) {
  return std::sqrt(vec3_dot(v, v));
}

Vec3 vec3_normalize(const Vec3& v) {
  const double n = vec3_norm(v);
  if (n < 1e-12) {
    return {1.0, 0.0, 0.0};
  }
  return vec3_scale(v, 1.0 / n);
}

Quat quat_mul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Quat quat_normalize(const Quat& q) {
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}

Quat quat_conj(const Quat& q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Vec3 quat_rotate(const Quat& q_in, const Vec3& v) {
  const Quat q = quat_normalize(q_in);
  const Quat p{0.0, v.x, v.y, v.z};
  const Quat t = quat_mul(quat_mul(q, p), quat_conj(q));
  return {t.x, t.y, t.z};
}

void quat_shortest_path(Quat* q) {
  if (q->w < 0.0) {
    q->w = -q->w;
    q->x = -q->x;
    q->y = -q->y;
    q->z = -q->z;
  } else if (q->w == 0.0) {
    if (q->x < 0.0 || (q->x == 0.0 && (q->y < 0.0 || (q->y == 0.0 && q->z < 0.0)))) {
      q->x = -q->x;
      q->y = -q->y;
      q->z = -q->z;
    }
  }
}

Quat quat_from_axis_angle(double axis_x, double axis_y, double axis_z, double angle_rad) {
  const double half = 0.5 * angle_rad;
  const double s = std::sin(half);
  const double c = std::cos(half);
  const double n = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
  if (n < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {c, axis_x / n * s, axis_y / n * s, axis_z / n * s};
}

Quat quat_yaw(double psi) {
  return quat_from_axis_angle(0.0, 0.0, 1.0, psi);
}

Quat quat_pitch(double theta) {
  return quat_from_axis_angle(0.0, 1.0, 0.0, theta);
}

Quat quat_exp(const Vec3& omega, double dt) {
  const double angle = vec3_norm(omega) * dt;
  if (angle < 1e-12) {
    return {1.0, 0.5 * omega.x * dt, 0.5 * omega.y * dt, 0.5 * omega.z * dt};
  }
  const double inv = 1.0 / vec3_norm(omega);
  return quat_from_axis_angle(omega.x * inv, omega.y * inv, omega.z * inv, angle);
}

Mat3 quat_to_mat3(const Quat& q_in) {
  const Quat q = quat_normalize(q_in);
  const double w = q.w;
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  Mat3 R{};
  R.m[0] = 1 - 2 * (y * y + z * z);
  R.m[1] = 2 * (x * y - w * z);
  R.m[2] = 2 * (x * z + w * y);
  R.m[3] = 2 * (x * y + w * z);
  R.m[4] = 1 - 2 * (x * x + z * z);
  R.m[5] = 2 * (y * z - w * x);
  R.m[6] = 2 * (x * z - w * y);
  R.m[7] = 2 * (y * z + w * x);
  R.m[8] = 1 - 2 * (x * x + y * y);
  return R;
}

Vec3 camera_boresight_world(const Quat& q_world_from_camera) {
  const Vec3 x_cam{1.0, 0.0, 0.0};
  return quat_rotate(q_world_from_camera, x_cam);
}

Vec3 boresight_error_camera(const Vec3& b_est, const Vec3& b_des) {
  return vec3_cross(b_est, b_des);
}

void inertial_to_joint_rates(double theta, const Vec3& omega_cam, double* theta_dot,
                             double* psi_dot) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  *theta_dot = omega_cam.y;
  *psi_dot = -s * omega_cam.x + c * omega_cam.z;
}

void camera_effort_to_torques(double theta, const Vec3& m_cam, double* tau_pitch,
                               double* tau_yaw) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  *tau_pitch = m_cam.y;
  *tau_yaw = -s * m_cam.x + c * m_cam.z;
}

bool is_finite(double x) {
  return std::isfinite(x);
}

bool vec3_finite(const Vec3& v) {
  return is_finite(v.x) && is_finite(v.y) && is_finite(v.z);
}

}  // namespace sgs
