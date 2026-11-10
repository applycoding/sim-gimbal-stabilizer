// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace sgs {
namespace sim {

class XorShift32 {
 public:
  explicit XorShift32(uint32_t seed);
  uint32_t next_u32();
  double uniform01();
  double gaussian();

 private:
  uint32_t state_;
};

}  // namespace sim
}  // namespace sgs
