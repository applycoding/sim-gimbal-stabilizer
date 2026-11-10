// SPDX-License-Identifier: Apache-2.0
#include "sgs/types.hpp"

#include <cstdlib>
#include <new>

namespace {

std::size_t alloc_count = 0;

void on_alloc() {
  ++alloc_count;
  std::abort();
}

}  // namespace

void* operator new(std::size_t sz) {
  on_alloc();
  return std::malloc(sz);
}
void* operator new[](std::size_t sz) {
  on_alloc();
  return std::malloc(sz);
}
void* operator new(std::size_t sz, const std::nothrow_t&) noexcept {
  on_alloc();
  return std::malloc(sz);
}
void* operator new[](std::size_t sz, const std::nothrow_t&) noexcept {
  on_alloc();
  return std::malloc(sz);
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }

int main() {
  sgs::Controller ctrl;
  sgs::ControllerState st{};
  sgs::Gains g = sgs::default_gains();
  sgs::Limits L = sgs::default_limits();
  sgs::reset_state(&st, {1, 0, 0, 0});

  sgs::StepInput in{};
  in.imu.flags = sgs::kFlagAccelValid | sgs::kFlagMagValid;
  in.imu.accel_m_s2[2] = -9.80665;
  in.imu.mag_uT[0] = 25;
  in.imu.mag_uT[2] = 40;
  in.enc.pitch_rad = 85.0 * 3.14159265358979323846 / 180.0;
  in.set.boresight_ned = {1, 0, 0};

  sgs::StepOutput out{};
  for (int i = 0; i < 2000; ++i) {
    ctrl.step(0.001, g, L, in, &st, &out);
    st.select = i % 2 == 0 ? sgs::ControllerState::EstimatorSelect::Complementary
                           : sgs::ControllerState::EstimatorSelect::Ekf;
    in.imu.accel_m_s2[0] = (i > 500) ? 14.0 : 0.0;
  }
  return alloc_count == 0 ? 0 : 1;
}
