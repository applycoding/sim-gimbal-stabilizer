// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sgs/types.hpp"

namespace sgs {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

Vec3 vec3_add(const Vec3& a, const Vec3& b);
Vec3 vec3_sub(const Vec3& a, const Vec3& b);
Vec3 vec3_scale(const Vec3& v, double s);
double vec3_dot(const Vec3& a, const Vec3& b);
Vec3 vec3_cross(const Vec3& a, const Vec3& b);
double vec3_norm(const Vec3& v);
Vec3 vec3_normalize(const Vec3& v);

Quat quat_mul(const Quat& a, const Quat& b);
Quat quat_normalize(const Quat& q);
Quat quat_conj(const Quat& q);
Vec3 quat_rotate(const Quat& q, const Vec3& v);
void quat_shortest_path(Quat* q);
Quat quat_from_axis_angle(double axis_x, double axis_y, double axis_z, double angle_rad);
Quat quat_yaw(double psi);
Quat quat_pitch(double theta);
Quat quat_exp(const Vec3& omega, double dt);

Vec3 camera_boresight_world(const Quat& q_world_from_camera);
Vec3 boresight_error_camera(const Vec3& b_est, const Vec3& b_des);

// Joint rates from camera inertial gyro: theta_dot = wy, psi_dot = -sin(theta)*wx + cos(theta)*wz
void inertial_to_joint_rates(double theta, const Vec3& omega_cam, double* theta_dot,
                             double* psi_dot);
// Torque map: tau_pitch = m_y, tau_yaw = -sin(theta)*m_x + cos(theta)*m_z
void camera_effort_to_torques(double theta, const Vec3& m_cam, double* tau_pitch,
                              double* tau_yaw);

Mat3 quat_to_mat3(const Quat& q);

bool is_finite(double x);
bool vec3_finite(const Vec3& v);

}  // namespace sgs
