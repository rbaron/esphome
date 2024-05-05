#pragma once

#include <cstdint>
#include <cstdlib>

namespace esphome {
namespace fusb302 {

// Timers. All in milliseconds.
constexpr int tTypeCSinkWaitCap = 465;
constexpr int tSourceEPRKeepAlive = 750;
constexpr int tPPSTimerIntervalMs = 7000;
constexpr int tSoftResetWatchdogIntervalMs = 1000;

// Sizes.
constexpr size_t kMaxPDOS = 7;
constexpr size_t kMaxExtendedMsgLen = 260;

// Control message types.
constexpr uint8_t kCtrlMsgTypeGoodCRC = 0b0001;
constexpr uint8_t kCtrlMsgTypeAccept = 0b0011;
constexpr uint8_t kCtrlMsgTypePSReady = 0b0110;
constexpr uint8_t kCtrlMsgTypeSoftReset = 0b1101;

// Data message types.
constexpr uint8_t kDataMsgTypeSourceCapabilities = 0b0001;
constexpr uint8_t kDataMsgTypeRequest = 0b0010;
constexpr uint8_t kDataMsgTypeEPRRequest = 0b1001;
constexpr uint8_t kDataMsgTypeEPRMode = 0b1010;

// Extended message types.
constexpr uint8_t kExtMsgTypeExtendedControl = 0b10000;
constexpr uint8_t kExtMsgTypeEPRSourceCapabilities = 0b10001;

// Extended control message types.
constexpr uint8_t kExtMsgTypeEPRKeepAlive = 0x3;
constexpr uint8_t kExtMsgTypeEPRKeepAliveAck = 0x4;

// FUSB302 fifo tokens.
constexpr uint8_t kTokSOP1 = 0x12;
constexpr uint8_t kTokSOP2 = 0x13;
constexpr uint8_t kTokEOP = 0x14;
constexpr uint8_t kTokPACKSYM = 0x80;
constexpr uint8_t kTokTXOFF = 0xfe;
constexpr uint8_t kTokTXON = 0xa1;

}  // namespace fusb302
}  // namespace esphome