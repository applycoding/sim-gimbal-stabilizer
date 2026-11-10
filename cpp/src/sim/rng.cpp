// SPDX-License-Identifier: Apache-2.0
#include "sgs/sim/rng.hpp"

#include <cmath>

namespace sgs {
namespace sim {

XorShift32::XorShift32(uint32_t seed) {
  state_ = seed == 0 ? 1u : seed;
}

uint32_t XorShift32::next_u32() {
  uint32_t x = state_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state_ = x;
  return x;
}

double XorShift32::uniform01() {
  return (next_u32() + 1u) / 4294967296.0;
}

double XorShift32::gaussian() {
  double u1 = uniform01();
  double u2 = uniform01();
  if (u1 < 1e-12) {
    u1 = 1e-12;
  }
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
}

}  // namespace sim
}  // namespace sgs
