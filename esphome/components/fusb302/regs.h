#pragma once

#include <cstdint>

namespace esphome {
namespace fusb302 {

constexpr uint8_t REG_DEVICE_ID = 0x01;

constexpr uint8_t REG_SWITCHES0 = 0x02;
constexpr uint8_t REG_SWITCHES1 = 0x03;
constexpr uint8_t REG_MEASURE = 0x04;
constexpr uint8_t REG_SLICE = 0x05;
constexpr uint8_t REG_CONTROL0 = 0x06;
constexpr uint8_t REG_CONTROL1 = 0x07;
constexpr uint8_t REG_CONTROL2 = 0x08;
constexpr uint8_t REG_CONTROL3 = 0x09;
constexpr uint8_t REG_MASK1 = 0x0A;
constexpr uint8_t REG_POWER = 0x0B;
constexpr uint8_t REG_RESET = 0x0C;
constexpr uint8_t REG_OCPREG = 0x0D;
constexpr uint8_t REG_MASKA = 0x0E;
constexpr uint8_t REG_MASKB = 0x0F;
constexpr uint8_t REG_CONTROL4 = 0x10;
constexpr uint8_t REG_STATUS0A = 0x3C;
constexpr uint8_t REG_STATUS1A = 0x3D;
constexpr uint8_t REG_INTERRUPTA = 0x3E;
constexpr uint8_t REG_INTERRUPTB = 0x3F;
constexpr uint8_t REG_STATUS0 = 0x40;
constexpr uint8_t REG_STATUS1 = 0x41;
constexpr uint8_t REG_INTERRUPT = 0x42;
constexpr uint8_t REG_FIFOS = 0x43;

}  // namespace fusb302
}  // namespace esphome