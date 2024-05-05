#pragma once

#include <cstdint>

namespace esphome {
namespace fusb302 {

// All timers are in milliseconds.
constexpr int tTypeCSinkWaitCap = 465;
constexpr int tSourceEPRKeepAlive = 750;
// constexpr int tSourceEPRKeepAlive = 200;

const size_t kMaxExtendedMsgLen = 260;

}  // namespace fusb302
}  // namespace esphome