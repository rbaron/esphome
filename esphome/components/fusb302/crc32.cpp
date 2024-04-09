#include "esphome/components/fusb302/crc32.h"

#include <stdint.h>
#include <stdlib.h>

namespace esphome {
namespace fusb302 {
namespace {

uint32_t crc32_table[256];
bool crc32_table_initialized = false;

// From https://wiki.osdev.org/CRC32.
void crc32_fill(uint32_t *table) {
  uint8_t index = 0, z;
  do {
    table[index] = index;
    for (z = 8; z; z--)
      table[index] = (table[index] & 1) ? (table[index] >> 1) ^ 0xEDB88320 : table[index] >> 1;
  } while (++index);
}

}  // namespace

// From https://wiki.osdev.org/CRC32.
uint32_t crc32(const uint8_t *data, size_t len) {
  if (!crc32_table_initialized) {
    crc32_fill(crc32_table);
    crc32_table_initialized = true;
  }

  uint32_t crc = 0xffffffff;
  while (len-- != 0)
    crc = crc32_table[((uint8_t) crc ^ *(data++))] ^ (crc >> 8);
  return (crc ^ 0xffffffff);
}

}  // namespace fusb302
}  // namespace esphome