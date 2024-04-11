#include "esphome/components/fusb302/fusb302.h"

#include "esphome/core/log.h"
#include "esphome/components/fusb302/pdo.h"
#include "esphome/components/fusb302/regs.h"
#include "esphome/components/fusb302/timers.h"
#include "esphome/components/fusb302/crc32.h"
#include "esphome/core/hal.h"
#include "Wire.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// #define HAS_BITS(v, b, n) (((v) >> (b)) & ((1 << (n)) - 1))
// #define HAS_BIT(v, b) (HAS_BITS(v, b, 1))

#define FUSB302_FAIL(str, ...) \
  do { \
    ESP_LOGE(TAG, str, ##__VA_ARGS__); \
    enter_state(State::FAILURE); \
  } while (0)

#define FUSB302_RETRY(expr, expected_ret) \
  do { \
    for (uint8_t i = 0; i < kI2CMaxTries; i++) { \
      if (expr == expected_ret) { \
        return true; \
      } \
      delay(5); \
      return false; \
    } \
  } while (0)

namespace esphome {
namespace fusb302 {

static const char *TAG = "fusb302.component";

constexpr uint8_t kI2CMaxTries = 3;
constexpr uint8_t kI2CSLeepBetweenAttemptsMS = 100;

namespace {

constexpr char kWaitForCapsTimerName[] = "wait_for_caps";
constexpr char kPPSTimerName[] = "pps_timer";
constexpr int kPPSTimerIntervalMs = 7000;

constexpr char kSoftResetWatchdogTimerName[] = "soft_watchdog";
constexpr int kSoftResetWatchdogIntervalMs = 250;

void dump_rdo(uint32_t *rdo_data, const PDO *pdos) {
  uint8_t obj_pos = (*rdo_data >> 28) & 0x7;
  uint8_t pdo_idx = obj_pos - 1;

  if (pdos[pdo_idx].type == PDO::Type::FIXED) {
    uint8_t give_back_flag = (*rdo_data >> 27) & 0x1;
    uint8_t cap_mis = (*rdo_data >> 26) & 0x1;
    uint8_t usb_cap = (*rdo_data >> 25) & 0x1;
    uint8_t no_usb_sus = (*rdo_data >> 24) & 0x1;
    uint8_t unchunked_msg_sup = (*rdo_data >> 23) & 0x1;
    uint8_t epr_cap = (*rdo_data >> 22) & 0x1;
    uint16_t curr_ma = (*rdo_data >> 10) & ((1 << 11) - 1);
    uint16_t max_curr_ma = (*rdo_data >> 0) & ((1 << 11) - 1);

    ESP_LOGD(TAG,
             "FIXED Object position: %d, give back flag: %d, cap mismatch: %d, USB cap: %d, no USB suspend: %d, "
             "unchunked msg sup: %d, EPR cap: %d, current: %d mA, max current: %d mA",
             obj_pos, give_back_flag, cap_mis, usb_cap, no_usb_sus, unchunked_msg_sup, epr_cap, curr_ma * 10,
             max_curr_ma * 10);
    return;
  } else if (pdos[pdo_idx].type == PDO::Type::AUGMENTED &&
             pdos[pdo_idx].augmented.type == PDO::Augmented::Type::SPR_PPS) {
    uint8_t should_be_zero1 = (*rdo_data >> 27) & 0x1;
    uint8_t cap_mis = (*rdo_data >> 26) & 0x1;
    uint8_t usb_cap = (*rdo_data >> 25) & 0x1;
    uint8_t no_usb_sus = (*rdo_data >> 24) & 0x1;
    uint8_t unchunked_msg_sup = (*rdo_data >> 23) & 0x1;
    uint8_t epr_cap = (*rdo_data >> 22) & 0x1;
    uint8_t should_be_zero2 = (*rdo_data >> 21) & 0x1;
    uint16_t voltage = (*rdo_data >> 9) & ((1 << 13) - 1);
    uint8_t should_be_zero3 = (*rdo_data >> 7) & ((1 << 3) - 1);
    uint16_t current = (*rdo_data >> 0) & ((1 << 8) - 1);

    ESP_LOGD(TAG,
             "PPS Object position: %d, cap mismatch: %d, USB cap: %d, no USB suspend: %d, unchunked msg sup: %d, "
             "EPR cap: %d, voltage: %d mV, current: %d mA, should be zero: %d %d %d",
             obj_pos, cap_mis, usb_cap, no_usb_sus, unchunked_msg_sup, epr_cap, voltage * 20, current * 50,
             should_be_zero1, should_be_zero2, should_be_zero3);
    return;
  }
  ESP_LOGW(TAG, "Unsupported RDO type: 0x%08X (PDO position %d)", *rdo_data, obj_pos);
}

uint32_t make_fixed_rdo(uint8_t pdo_idx, uint16_t max_current_ma) {
  uint32_t rdo = 0;
  rdo |= ((pdo_idx + 1) << 28);  // Object position.
  rdo |= (0x0 << 27);            // Give back flag.
  rdo |= (0x0 << 26);            // Cap mismatch.
  rdo |= (0x1 << 25);            // USB cap.
  rdo |= (0x1 << 24);            // No USB suspend.
  rdo |= (0x0 << 23);            // Unchunked message supported.
  rdo |= (0x0 << 22);            // EPR cap.
  rdo |= (max_current_ma / 10) << 10;
  rdo |= (max_current_ma / 10) << 0;
  return rdo;
}

uint32_t make_pps_rdo(uint8_t pdo_idx, uint16_t voltage_mv, uint16_t current_ma) {
  uint32_t rdo = 0;
  rdo |= ((pdo_idx + 1) << 28);  // Object position.
  rdo |= (0x0 << 26);            // Cap mismatch.
  // rdo |= (0x1 << 25);            // USB cap.
  // rdo |= (0x1 << 24);            // No USB suspend.
  // rdo |= (0x0 << 23);            // Unchunked message supported.
  // rdo |= (0x0 << 22);  // EPR cap.
  rdo |= (voltage_mv / 20) << 9;
  rdo |= (current_ma / 50) << 0;
  return rdo;
}

}  // namespace

void FUSB302::dump_config() {
  ESP_LOGCONFIG(TAG, "fusb302");
  LOG_UPDATE_INTERVAL(this);
}

bool FUSB302::measure_cc_pin(uint8_t cc_pin, uint8_t *voltage_out) {
  // Enable CCx measuring circuit.
  if (!this->write_byte(REG_SWITCHES0, 0x03 | (cc_pin << 2))) {
    FUSB302_FAIL("Failed to write to SWITCHES0");
    return false;
  }

  // Wait for the measurement to complete.
  delay(50);

  // Read the voltage.
  if (!this->read_byte(REG_STATUS0, voltage_out)) {
    FUSB302_FAIL("Failed to read voltage");
    return false;
  }
  // We only care for the lower 2 bits.
  *voltage_out &= 0b11;
  return true;
}

void FUSB302::setup() {
  // Set up the interrupt pin.
  if (int_pin_ != nullptr) {
    // FUSB302_FAIL("Required interrupt pin not set");
    // return;
    ESP_LOGD(TAG, "Setting up interrupt pin");
    int_pin_->setup();
    int_pin_->attach_interrupt(FUSB302::ISR, this, gpio::INTERRUPT_FALLING_EDGE);
  }

  // Reset.
  if (!this->write_byte(REG_RESET, 0x03)) {
    FUSB302_FAIL("Failed to write to RESET");
    return;
  }

  // Write to POWER.
  if (!this->write_byte(REG_POWER, 0x0f)) {
    FUSB302_FAIL("Failed to write to POWER");
    return;
  }

  // Get the device id.
  uint8_t device_id;
  if (this->read_register(REG_DEVICE_ID, (uint8_t *) &device_id, 1, false)) {
    FUSB302_FAIL("Failed to read device id");
    return;
  } else {
    ESP_LOGV(TAG, "Device id: 0x%04X", device_id);
  }

  // Mask all these interrupts.
  if (!this->write_byte(REG_MASK1, 0xff)) {
    FUSB302_FAIL("Failed to write to MASK1");
    return;
  }

  // Enable all these interrupts.
  if (!this->write_byte(REG_MASKA, 0x00)) {
    FUSB302_FAIL("Failed to write to MASKA");
    return;
  }
  if (!this->write_byte(REG_MASKB, 0x00)) {
    FUSB302_FAIL("Failed to write to MASKB");
    return;
  }
  if (!this->write_byte(REG_CONTROL0, 0b11 << 2)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL0");
    FUSB302_FAIL("Failed to write to CONTROL0");
    return;
  }

  // Enable packet retry.
  if (!this->write_byte(REG_CONTROL3, 0x07)) {
    FUSB302_FAIL("Failed to write to CONTROL3");
    return;
  }

  if (!this->write_byte(REG_CONTROL2, 0x00)) {
    FUSB302_FAIL("Failed to write to CONTROL2");
    return;
  }

  // Clear RX fifo.
  if (!this->write_byte(REG_CONTROL1, 0x04)) {
    FUSB302_FAIL("Failed to write to CONTROL1");
    return;
  }

  // Ready CC1 and CC2 voltages to figure out which one we're connected to.
  uint8_t cc1_voltage, cc2_voltage;
  if (!this->measure_cc_pin(/*cc_pin=*/1, &cc1_voltage)) {
    FUSB302_FAIL("Failed to read CC1 voltage");
    return;
  }
  if (!this->measure_cc_pin(/*cc_pin=*/2, &cc2_voltage)) {
    FUSB302_FAIL("Failed to read CC2 voltage");
    return;
  }
  ESP_LOGD(TAG, "CC1 voltage: %d, CC2 voltage: %d", cc1_voltage, cc2_voltage);

  uint8_t cc_pin = cc1_voltage > cc2_voltage ? 0b01 : 0b10;
  if (!this->write_byte(REG_SWITCHES0, 0x03 | (cc_pin << 2))) {
    FUSB302_FAIL("Failed to write to SWITCHES0");
    return;
  }

  // Enable auto GoodCRC response and TXCCx, plus keep the revision 2.0 of the GoodCRC ack packet.
  if (!this->write_byte(REG_SWITCHES1, cc_pin | (1 << 2) | (1 << 5))) {
    FUSB302_FAIL("Failed to write to SWITCHES1");
    return;
  }

  // Flush the TX FIFO.
  if (!this->write_byte(REG_CONTROL0, 0x44)) {
    FUSB302_FAIL("Failed to write to CONTROL0");
    return;
  }
  // Flush the RX FIFO.
  if (!this->write_byte(REG_CONTROL1, 0x1 << 2)) {
    FUSB302_FAIL("Failed to write to CONTROL1");
    return;
  }

  // Reset PD logic.
  if (!this->write_byte(REG_RESET, 0x02)) {
    FUSB302_FAIL("Failed to write to RESET");
    return;
  }

  // We have to be as fast as we can during the negotiation phase, so we'll use a high frequency loop. We will disable
  // it once the negotiation is complete.
  high_freq_loop_req_.start();

  enter_state(State::WAIT_FOR_CAPABILITIES);

  // xTaskCreate(FUSB302::task, "fusb302_task", 4096, this, 1, nullptr);
  // xTaskCreatePinnedToCore(FUSB302::task, "fusb302_task", 4 * 4096, this, 1, nullptr, 1);

  // xTaskCre

  this->set_timeout(kSoftResetWatchdogTimerName, kSoftResetWatchdogIntervalMs, [this]() { FUSB302::Watchdog(this); });
}

void FUSB302::loop() {
  LockGuard lock(mutex_);
  // Continuously poll if no interrupt pin is set, or if an interrupt is pending.
  if (this->int_pin_ == nullptr || this->interrupt_pending_) {
    this->interrupt_pending_ = false;
    this->process_interrupt();
    ESP_LOGD(TAG, "Processed interrupt");
  }
}

void FUSB302::update() {}

// Interrupt callback.
void FUSB302::ISR(FUSB302 *instance) { instance->interrupt_pending_ = true; }

bool FUSB302::process_interrupt() {
  // Reading a lot here may cause timing issues.
  ESP_LOGD(TAG, "Processing interrupt");

  // Read interrupt registers -- this will clear the interrupt.
  // TODO: maybe we can read all of these registers at once.
  // uint8_t interrupt, interrupta, interruptb;
  // if (!this->read_byte_retry(REG_INTERRUPT, &interrupt)) {
  //   ESP_LOGE(TAG, "Failed to read interrupt");
  //   return false;
  // }
  // if (!this->read_byte_retry(REG_INTERRUPTA, &interrupta)) {
  //   ESP_LOGE(TAG, "Failed to read interrupta");
  //   return false;
  // }
  // if (!this->read_byte_retry(REG_INTERRUPTB, &interruptb)) {
  //   ESP_LOGE(TAG, "Failed to read interruptb");
  //   return false;
  // }

  // Read all interrupt registers.
  volatile uint8_t buf[7];
  i2c::ErrorCode err;
  if (err = this->read_register_retry(REG_STATUS0A, (uint8_t *) &buf, sizeof(buf), true)) {
    ESP_LOGE(TAG, "Failed to read reg. Error: %d", err);
    return false;
    // } else {
    //   ESP_LOGW(TAG, "Reg: 0x%04X", buf[0]);
  }

  // // Reset INT_MASK to clear the interrupt. This shouldn't be necessary?
  // if (!this->write_byte_retry(REG_CONTROL0, 0x01 << 5)) {
  //   ESP_LOGE(TAG, "Failed to clear INT_N");
  //   return false;
  // }
  // if (!this->write_byte_retry(REG_CONTROL0, 0x00 << 5)) {
  //   ESP_LOGE(TAG, "Failed to clear INT_N");
  //   return false;
  // }

  // IDEA: if reading FIFO fails, we could end up in an unpredictable state. Maybe we should reset the fifo altogether
  // and send a soft reset.
  while (this->has_fifo_msg()) {
    delay(1);
    // Read FIFO data into fifo_msg_.
    if (!this->read_fifo()) {
      ESP_LOGE(TAG, "Failed to read FIFO");
      return false;
    }

    // Handle the message if its for us. Under failure we still have to clear the FIFO, but we don't handle the
    // messages. If the FIFO overflows, the device will reset.
    if (state_ != State::FAILURE && fifo_msg_.destination == FIFOMsg::Destination::SOP && !handle_msg()) {
      ESP_LOGE(TAG, "Failed to handle message");
      return false;
    }
  }

  return true;
}

bool FUSB302::has_fifo_msg() {
  uint8_t status1;
  if (!this->read_byte_retry(REG_STATUS1, &status1)) {
    ESP_LOGE(TAG, "Failed to read status1");
    return false;
  }
  return (status1 & (1 << 5)) == 0;
}

bool FUSB302::read_fifo() {
  // RX token (1) + header (2) + MAX_PDOS * 4 + CRC (4).
  uint8_t buf[1 + 2 + 4 * FUSB302_MAX_PDOS + 4];
  if (this->read_register_retry(REG_FIFOS, buf, 1 + 2)) {
    ESP_LOGE(TAG, "Failed to read FIFO rx token and header");
    return false;
  }
  // ESP_LOGD(TAG, "Header: 0x%04X", header);
  fifo_msg_.header = buf[2] << 8 | buf[1];
  fifo_msg_.n_objs = (fifo_msg_.header >> 12) & 0x07;
  fifo_msg_.msg_type = fifo_msg_.header & 0xf;

  if (fifo_msg_.n_objs > FUSB302_MAX_PDOS) {
    ESP_LOGE(TAG, "Too many objects in message of type %d: %d", fifo_msg_.msg_type, fifo_msg_.n_objs);
    return false;
  }

  // uint8_t obj_buff[4 * FUSB302_MAX_PDOS + 4];
  if (this->read_register_retry(REG_FIFOS, buf + 3, 4 * fifo_msg_.n_objs + 4)) {
    ESP_LOGE(TAG, "Failed to read objects");
    return false;
  }

  // Write into fifo_msg_.
  for (uint8_t i = 0; i < fifo_msg_.n_objs; i++) {
    fifo_msg_.objs[i] =
        (buf[3 + 4 * i + 3] << 24) | (buf[3 + 4 * i + 2] << 16) | (buf[3 + 4 * i + 1] << 8) | buf[3 + 4 * i];
  }

  uint8_t rx_token = buf[0];
  fifo_msg_.destination = ((rx_token >> 4) & 0x0e) == 0x0e ? FIFOMsg::Destination::SOP : FIFOMsg::Destination::UNKNOWN;

  uint32_t crc = (buf[3 + 4 * fifo_msg_.n_objs + 3] << 24) | (buf[3 + 4 * fifo_msg_.n_objs + 2] << 16) |
                 (buf[3 + 4 * fifo_msg_.n_objs + 1] << 8) | buf[3 + 4 * fifo_msg_.n_objs];
  uint32_t crc_calc = crc32(buf + 1, 2 + 4 * fifo_msg_.n_objs);

  if (crc != crc_calc) {
    ESP_LOGE(TAG, "CRC mismatch! 0x%08X != 0x%08X", crc, crc_calc);
    return false;
  }

  // for (int i = 1; i < 3 + 4 * fifo_msg_.n_objs; i++) {
  //   ESP_LOGE(TAG, " 0x%02X,", buf[i]);
  // }
  // ESP_LOGE(TAG, " EOF,");

  return true;
}

// TODO: handle more messages;
// - Error?
// - Soft reset?
// - Hard reset?
// - Overheat?
bool FUSB302::handle_msg() {
  if (fifo_msg_.n_objs == 0) {
    // ESP_LOGD(TAG, "No objects in message -- command message. Type: 0x%02X", msg_type);
    if (fifo_msg_.msg_type == 0x01) {  // GoodCRC.
      return true;
    } else if (fifo_msg_.msg_type == 0x03) {  // Accept.
      ESP_LOGW(TAG, "Accept message received");
      enter_state(State::TRANSITION_SINK);
    } else if (fifo_msg_.msg_type == 0x06) {  // PS_RDY.
      ESP_LOGW(TAG, "PS_RDY message received");
      enter_state(State::READY);
    } else if (fifo_msg_.msg_type == 0x0d) {  // Soft_Reset.
      // We need to respond in less than 27ms, so no state transition. Send an Accept message right away.
      // If things go well, the source will send a Source_Capabilities message shortly after.
      if (!this->send_msg(0b11, 0, nullptr)) {
        ESP_LOGE(TAG, "Failed to send Accept in response to Soft_Reset");
        return false;
      }
      ESP_LOGE(TAG, "Soft_Reset received, replied with Accept");
    } else {
      ESP_LOGW(TAG, "Unhandled command message type: 0x%02X", fifo_msg_.msg_type);
    }
  } else {
    if (fifo_msg_.msg_type == 0x01) {  // Source_Capabilities.
      enter_state(State::EVALUATE_CAPABILITY);

      // Extract the PD spec revision.
      pd_spec_ = (fifo_msg_.header >> 6) & 0x3;

      // We have to be fast to send this response (ideally < 10ms). Otherwise the power supply will hard reset. So
      // better to avoid a state loop and just do it right away.
      if (!this->parse_pdos(fifo_msg_.n_objs, fifo_msg_.objs)) {
        ESP_LOGE(TAG, "Failed to parse PDOS");
        return false;
      }
      if (!this->request_pdo()) {
        return false;
      }
      enter_state(State::SELECT_CAPABILITY);
      // ESP_LOGW(TAG, "Requested PDO: ");
      log_pdo(pdos_[*selected_pdo_idx_]);
    } else {
      ESP_LOGW(TAG, "Unhandled data message type: 0x%02X", fifo_msg_.msg_type);
    }
  }
  return true;
}

bool FUSB302::parse_pdos(uint8_t n_pdos, uint32_t *pdos) {
  // Ensure we're starting clean.
  pdos_.clear();
  selected_pdo_idx_.reset();

  // Parse PDOs.
  for (uint8_t i = 0; i < n_pdos; i++) {
    pdos_.push_back(parse_pdo(pdos[i]));
    const PDO &pdo = pdos_.back();
    if (!pdo.parsed) {
      ESP_LOGE(TAG, "Failed to parse PDO 0x%08X", pdos[i]);
      return false;
    }
    // log_pdo(pdo);
    // Select the first compatible PDO (fixed should be listed first).
    if (!selected_pdo_idx_.has_value() && is_pdo_compatible(pdo, power_requirement_)) {
      selected_pdo_idx_ = i;
    }
  }

  if (!selected_pdo_idx_.has_value()) {
    // If we don't find a compatible PDO, we'll just select the first one, which is guaranteed to be Safe5V (I
    // think?).
    // TODO: check that we actually parsed the PDOs and that PDO 0 is indeed Safe5V.
    // TODO: move this logic to request_pdo.
    selected_pdo_idx_ = 0;
    // enter_state(State::REQUESTED_SAFE_5V);
  } else {
    // enter_state(State::REQUESTED_PDO);
  }
  return true;
}

bool FUSB302::request_pdo() {
  // Fixed or PPS?
  const PDO &pdo = pdos_[*selected_pdo_idx_];
  uint32_t request;
  if (pdo.type == PDO::Type::FIXED) {
    request = make_fixed_rdo(*selected_pdo_idx_, power_requirement_.current_ma);
  } else if (pdo.type == PDO::Type::AUGMENTED && pdo.augmented.type == PDO::Augmented::Type::SPR_PPS) {
    request = make_pps_rdo(*selected_pdo_idx_, power_requirement_.voltage_mv, power_requirement_.current_ma);
  } else {
    ESP_LOGE(TAG, "Unsupported PDO type: %d", static_cast<int>(pdo.type));
    return false;
  }

  if (!this->send_msg(0b10, sizeof(request), (uint8_t *) &request)) {
    ESP_LOGE(TAG, "Failed to send PD request");
    return false;
  }
  return true;
}

bool FUSB302::send_soft_reset() {
  // Try our best to flush the contents for RX and TX fifo.
  if (!this->write_byte_retry(REG_CONTROL0, 0x1 << 6)) {
    ESP_LOGE(TAG, "Failed to flush TX FIFO");
    return false;
  }
  if (!this->write_byte_retry(REG_CONTROL1, 0x1 << 2)) {
    ESP_LOGE(TAG, "Failed to flush TX FIFO");
    return false;
  }
  if (!this->send_msg(0b1101, 0, nullptr)) {
    ESP_LOGE(TAG, "Failed to send soft reset");
    return false;
  }
  return true;
}

bool FUSB302::send_msg(uint8_t msg_type, uint8_t len, uint8_t *data) {
  static uint8_t msg_id = 0;

  // Truncate to 3 bits (same as % 8).
  msg_id &= 0x7;

  // uint8_t last = 0x80 | (len + 2);
  // If we calculate CRC ourselves...
  uint8_t last = 0x80 | (2 + len + 4);
  const uint8_t sop[] = {0x12, 0x12, 0x12, 0x13, last};
  // const uint8_t eop[4] = {0xff, 0x14, 0xfe, 0xa1};
  const uint8_t eop[] = {0x14, 0xfe, 0xa1};

  uint16_t header = 0;
  // Number of objects -- 1.
  // header |= (0x1 << 12);
  header |= ((len / sizeof(uint32_t)) << 12);
  // Spec revision -- 2.0.
  // header |= (0x1 << 6);
  // Spec revision -- 3.0 (worked with PPS for all adapters).
  // header |= (0x1 << 7);
  // Use same spec revision as the source.
  header |= (pd_spec_ << 6);
  // Message type -- Request.
  // header |= (0x1 << 1);
  header |= msg_type;
  // Message ID.
  header |= ((msg_id++) << 9);

  // TODO: make this slightly less horrible.
  uint8_t buff[64];
  uint8_t pos = 0;
  memcpy(buff + pos, sop, sizeof(sop));
  pos += sizeof(sop);
  memcpy(buff + pos, (uint8_t *) &header, sizeof(header));
  pos += sizeof(header);
  memcpy(buff + pos, data, len);
  pos += len;
  uint32_t crc = crc32(buff + sizeof(sop), sizeof(header) + len);
  memcpy(buff + pos, &crc, sizeof(crc));
  pos += sizeof(crc);
  memcpy(buff + pos, eop, sizeof(eop));
  pos += sizeof(eop);

  // for (size_t i = sizeof(sop); i < sizeof(sop) + sizeof(header) + len + sizeof(crc); i++) {
  //   ESP_LOGE(TAG, " 0x%02X,", buff[i]);
  // }

  // ESP_LOGE(TAG, "Calculated CRC: 0x%08X", crc);

  // memcpy(buff, sop, sizeof(sop));
  // memcpy(buff + sizeof(sop), (uint8_t *) &header, sizeof(header));
  // memcpy(buff + sizeof(sop) + sizeof(header), data, len);
  // memcpy(buff + sizeof(sop) + sizeof(header) + len, eop, sizeof(eop));

  // if (this->write_register_retry(REG_FIFOS, buff, sizeof(sop) + sizeof(header) + len + sizeof(eop))) {
  if (this->write_register_retry(REG_FIFOS, buff, pos)) {
    ESP_LOGE(TAG, "Failed to write eop to FIFO");
    // return false;
  }
  // ESP_LOGD(TAG, "Sent message");
  return true;
}

void FUSB302::maybe_rerequest_pps_pdo() {
  LockGuard lock(mutex_);

  // Schedule a new soft reset watchdog timer.
  this->set_timeout(kSoftResetWatchdogTimerName, kSoftResetWatchdogIntervalMs, [this]() { FUSB302::Watchdog(this); });

  ESP_LOGW(TAG, "Maybe rerequesting PPS PDO. State is: %d", static_cast<int>(state_));
  // if (state_ == State::READY && selected_pdo_idx_.has_value() &&
  if (selected_pdo_idx_.has_value() && pdos_[*selected_pdo_idx_].type == PDO::Type::AUGMENTED &&
      pdos_[*selected_pdo_idx_].augmented.type == PDO::Augmented::Type::SPR_PPS) {
    ESP_LOGW(TAG, "Ok! Re-requesting PPS PDO.");
    this->request_pdo();

    enter_state(State::SELECT_CAPABILITY);

    ESP_LOGW(TAG, "Done re-requesting PDO. Re-scheduling PPS timer");
    // Schedule a new PPS timer.
    this->set_timeout(kPPSTimerName, kPPSTimerIntervalMs, [this]() { this->maybe_rerequest_pps_pdo(); });
  } else {
    ESP_LOGE(TAG, "Not in PPS mode!");
  }
}

void FUSB302::enter_state(State state) {
  if (state_ == state) {
    return;
  }

  state_ = state;

  switch (state_) {
    case State::WAIT_FOR_CAPABILITIES: {
      // Start hard reset timer.
      this->set_timeout(kWaitForCapsTimerName, tTypeCSinkWaitCap, [this]() {
        ESP_LOGE(TAG, "Timed out waiting for capabilities. Issuing a hard reset.");
        for (uint8_t i = 0; i < 3; i++) {
          if (this->write_byte_retry(REG_CONTROL3, 0b1 << 6)) {
            ESP_LOGE(TAG, "Hard reset sent.");
            return;
          }
          delay(25);
        }
        ESP_LOGE(TAG, "Unable to request hard reset. Nothing else I can do :(");
      });
      break;
    }
    case State::EVALUATE_CAPABILITY: {
      // Stop hard reset timer.
      cancel_timeout(kWaitForCapsTimerName);
      break;
    }
    case State::READY: {
      // We can probably go back to usual loop update frequency.
      high_freq_loop_req_.stop();

      // Is this the voltage we wanted or a fallback Safe5V?
      if (selected_pdo_idx_.has_value() && is_pdo_compatible(pdos_[*selected_pdo_idx_], power_requirement_)) {
        on_pd_negotiation_success_callback_.call(/*success=*/true);
      } else {
        on_pd_negotiation_failure_callback_.call(/*success=*/false);
        ESP_LOGE(TAG,
                 "No compatible PDO found for voltage: %u mV; current: %u mA. Negotiated the fallback safe 5V. The "
                 "available Power Delivery Objects are: ",
                 power_requirement_.voltage_mv, power_requirement_.current_ma);
        for (const auto &pdo : pdos_) {
          log_pdo(pdo);
        }
        return;
      }

      // Do we need to schedule a PPS timer?
      cancel_timeout(kPPSTimerName);
      if (selected_pdo_idx_.has_value() && pdos_[*selected_pdo_idx_].type == PDO::Type::AUGMENTED &&
          pdos_[*selected_pdo_idx_].augmented.type == PDO::Augmented::Type::SPR_PPS) {
        this->set_timeout(kPPSTimerName, kPPSTimerIntervalMs, [this]() { this->maybe_rerequest_pps_pdo(); });
      }
      break;
    }
    case State::FAILURE: {
      on_pd_negotiation_failure_callback_.call(/*success=*/false);
      cancel_timeout(kPPSTimerName);
      break;
    }
    default:
      break;
  }
}

i2c::ErrorCode FUSB302::read_register_retry(uint8_t a_register, uint8_t *data, size_t len, bool stop) {
  // return this->read_register(a_register, data, len, stop);
  i2c::ErrorCode error;
  for (uint8_t i = 0; i < kI2CMaxTries; i++) {
    if ((error = this->read_register(a_register, data, len, stop)) == i2c::ErrorCode::ERROR_OK) {
      break;
    }
    ESP_LOGW(TAG, "Failed to read register 0x%02x with error %d. Retrying...", a_register, error);
    delay(kI2CSLeepBetweenAttemptsMS);
  }
  if (error != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGE(TAG, "Exceeded max retries for reading register 0x%02x. Giving up.", a_register);
  }
  delay(1);
  return error;

  // Wire.beginTransmission(0x22);
  // Wire.write(a_register);
  // Wire.endTransmission();
  // Wire.requestFrom(0x22, len);
  // while (Wire.available() && len > 0) {
  //   *data++ = Wire.read();
  //   len--;
  // }
  // return len == 0 ? i2c::ERROR_OK : i2c::ERROR_UNKNOWN;
}

i2c::ErrorCode FUSB302::write_register_retry(uint8_t a_register, const uint8_t *data, size_t len, bool stop) {
  // return this->write_register(a_register, data, len, stop);
  i2c::ErrorCode error;
  for (uint8_t i = 0; i < kI2CMaxTries; i++) {
    if ((error = this->write_register(a_register, data, len, stop)) == i2c::ErrorCode::ERROR_OK) {
      break;
    }
    ESP_LOGW(TAG, "Failed to write register 0x%02x with error %d. Retrying...", a_register, error);
    delay(kI2CSLeepBetweenAttemptsMS);
  }
  if (error != i2c::ErrorCode::ERROR_OK) {
    ESP_LOGE(TAG, "Exceeded max retries for writing register 0x%02x. Giving up.", a_register);
  }
  delay(1);
  return error;

  // Wire.beginTransmission(0x22);
  // Wire.write(a_register);
  // while (len > 0) {
  //   Wire.write(*data++);
  //   len--;
  // }
  // Wire.endTransmission();
  // return i2c::ERROR_OK;
}

bool FUSB302::read_byte_retry(uint8_t reg, uint8_t *value, bool stop) {
  return read_register_retry(reg, value, 1, stop) == i2c::ErrorCode::ERROR_OK;
}

bool FUSB302::write_byte_retry(uint8_t reg, uint8_t value, bool stop) {
  return write_register_retry(reg, &value, 1, stop) == i2c::ErrorCode::ERROR_OK;
}

void FUSB302::task(void *arg) {
  FUSB302 *instance = static_cast<FUSB302 *>(arg);
  // LockGuard lock(instance->mutex_);
  // // if (state_ == State::READY) {
  // //   delay(1);
  // // }
  // // Continuously poll if no interrupt pin is set, or if an interrupt is pending.
  while (true) {
    if (instance->int_pin_ == nullptr || instance->interrupt_pending_) {
      LockGuard lock(instance->mutex_);
      instance->interrupt_pending_ = false;
      instance->process_interrupt();
      ESP_LOGD(TAG, "Processed interrupt");
    }
  }
  // ESP_LOGE(TAG, "Task started!");
  // while (true) {
  // }
}

void FUSB302::Watchdog(FUSB302 *instance) {
  LockGuard lock(instance->mutex_);

  // If we're in failure, we can't do anything.
  if (instance->state_ == State::FAILURE) {
    return;
  }

  // We will try to fix the issue first, otherwise we will revisit the watchdog.
  instance->set_timeout(kSoftResetWatchdogTimerName, kSoftResetWatchdogIntervalMs,
                        [instance]() { instance->Watchdog(instance); });

  // If interrupt is asserted, something is wrong. Could be caused by an error in clearing the interrupt.
  if (instance->int_pin_ != nullptr && instance->int_pin_->digital_read() == LOW) {
    ESP_LOGW(TAG, "Interrupt is asserted. Something is wrong.");
    // This will re-clear the interrupt.
    instance->process_interrupt();
    return;
  }

  // We're good if we're in READY state.
  if (instance->soft_reset_test_-- < 0 && instance->state_ == State::READY) {
    ESP_LOGW(TAG, "Watchdog expired, but we're in READY state. Not doing anything.");
    instance->cancel_timeout(kSoftResetWatchdogTimerName);
    return;
  }

  // Send a soft reset.
  ESP_LOGW(TAG, "Watchdog expected READY state -- sending a soft reset (test: %d).", instance->soft_reset_test_);

  for (uint8_t i = 0; i < kI2CMaxTries; i++) {
    if (instance->send_soft_reset()) {
      ESP_LOGE(TAG, "Soft reset sent.");
      return;
    }
    ESP_LOGE(TAG, "Error sending soft reset.");
    delay(5);
  }

  // if (instance->state_ == State::FAILURE) {
  //   return;
  // }
  // ESP_LOGW(TAG, "Soft reset watchdog expired. Issuing a soft reset.");
  // for (uint8_t i = 0; i < 3; i++) {
  //   if (instance->write_byte_retry(REG_CONTROL3, 0b1 << 7)) {
  //     ESP_LOGE(TAG, "Soft reset sent.");
  //     return;
  //   }
  //   delay(25);
  // }
  // ESP_LOGE(TAG, "Unable to request soft reset. Nothing else I can do :(");
}

}  // namespace fusb302
}  // namespace esphome
