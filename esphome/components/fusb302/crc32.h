#pragma once

#include <stdint.h>
#include <stdlib.h>

namespace esphome {
namespace fusb302 {

uint32_t crc32(const uint8_t *data, size_t len);

}  // namespace fusb302
}  // namespace esphome