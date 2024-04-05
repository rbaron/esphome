#include "esphome/components/fusb302/fusb302.h"

#include "esphome/core/log.h"
#include "esphome/components/fusb302/pdo.h"
#include "esphome/components/fusb302/regs.h"
#include "esphome/components/fusb302/timers.h"
#include "esphome/core/hal.h"

// #define HAS_BITS(v, b, n) (((v) >> (b)) & ((1 << (n)) - 1))
// #define HAS_BIT(v, b) (HAS_BITS(v, b, 1))

#define FUSB302_FAIL(str, ...) \
  do { \
    ESP_LOGE(TAG, str, ##__VA_ARGS__); \
    enter_state(State::FAILURE); \
  } while (0)

namespace esphome {
namespace fusb302 {

static const char *TAG = "fusb302.component";

namespace {

constexpr char kWaitForCapsTimerName[] = "wait_for_caps";
constexpr char kPPSTimerName[] = "pps_timer";
constexpr int kPPSTimerIntervalMs = 8000;

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
  // Reset.
  // if (!this->write_byte(REG_RESET, 0x01)) {
  if (!this->write_byte(REG_RESET, 0x03)) {
    FUSB302_FAIL("Failed to write to RESET");
    return;
  }

  // Disconnect pull downs.
  // if (!this->write_byte(REG_SWITCHES0, 0x00)) {
  //   FUSB302_FAIL("Failed to write to SWITCHES0");
  //   return;
  // }

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

  // Enable all interrupts.
  if (!this->write_byte(REG_MASK1, 0x00)) {
    FUSB302_FAIL("Failed to write to MASK1");
    return;
  }
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

  // We know by design that we're connected to CC1. For a general approach this has to be detected.
  // Enable CC1 measuring circuit.
  // if (!this->write_byte(REG_SWITCHES0, 0x07)) {
  //   FUSB302_FAIL("Failed to write to SWITCHES0");
  //   return;
  // }
  uint8_t cc1_voltage, cc2_voltage;
  if (!this->measure_cc_pin(/*cc_pin=*/1, &cc1_voltage)) {
    FUSB302_FAIL("Failed to read CC1 voltage");
    return;
  }
  if (!this->measure_cc_pin(/*cc_pin=*/2, &cc2_voltage)) {
    FUSB302_FAIL("Failed to read CC2 voltage");
    return;
  }
  ESP_LOGW(TAG, "CC1 voltage: %d, CC2 voltage: %d", cc1_voltage, cc2_voltage);

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

  // Enable auto GoodCRC response and TXCC1, plus keep the revision 2.0 of the GoodCRC ack packet.
  // if (!this->write_byte(REG_SWITCHES1, (1 << 0) | (1 << 2) | (1 << 5))) {
  //   FUSB302_FAIL("Failed to write to SWITCHES1");
  //   return;
  // }

  // Figure out which CC we're connected to.
  // uint8_t cc1_voltage, cc2_voltage;

  // // Enable CC1 measuring circuit.
  // if (!this->write_byte(REG_SWITCHES0, 0x07)) {
  //   FUSB302_FAIL("Failed to write to SWITCHES0");
  //   return;
  // }

  // Reset PD logic.
  if (!this->write_byte(REG_RESET, 0x02)) {
    FUSB302_FAIL("Failed to write to RESET");
    return;
  }

  // Send a Get_Source_Cap message.
  // uint32_t get_source_cap = 0;
  // get_source_cap |= (0x1 << 12);  // Number of objects.
  // get_source_cap |= (0x1 << 1);   // Message type.
  // if (!this->send_msg(sizeof(get_source_cap), (uint8_t *) &get_source_cap)) {
  //   FUSB302_FAIL("Failed to send Get_Source_Cap message");
  //   return;
  // }

  // We have to be as fast as we can during the negotiation phase, so we'll use a high frequency loop. We will disable
  // it once the negotiation is complete.
  high_freq_loop_req_.start();

  enter_state(State::WAIT_FOR_CAPABILITIES);
}

void FUSB302::loop() {
  if (!this->process_interrupt()) {
    return;
  }
}

void FUSB302::update() {}

// Interrupt callback.
// TODO: set up interrupt handling. Right now we're just polling.
void FUSB302::ISR(FUSB302 *instance) { instance->interrupt_pending_ = true; }

bool FUSB302::process_interrupt() {
  // Reading a lot here may cause timing issues.

  // Read interrupt.
  // uint8_t interrupt;
  // if (this->read_register(REG_INTERRUPT, &interrupt, 1)) {
  //   ESP_LOGW(TAG, "Failed to read interrupt");
  //   return false;
  // }
  // ESP_LOGD(TAG, "Interrupt: 0x%02X", interrupt);

  // uint8_t status0;
  // if (this->read_register(REG_STATUS0, &status0, 1)) {
  //   ESP_LOGW(TAG, "Failed to read status0");
  //   return false;
  // }

  uint8_t status1;
  if (this->read_register(REG_STATUS1, &status1, 1)) {
    ESP_LOGW(TAG, "Failed to read status1");
    return false;
  }
  // If there's no data to read, we're done.
  if ((status1 & (1 << 5)) != 0) {
    return true;
  }

  // Read FIFO data into fifo_msg_.
  if (!this->read_fifo()) {
    FUSB302_FAIL("Failed to read FIFO");
    return false;
  }

  // We still better clear the FIFO even in failure mode, otherwise reset may occur. THis seem to happen for power
  // adapters that send a lot of vendor defined messages. An alternative would be to set the BIST_MODE bit in the
  // CONTROL1 register, which flushes the RX buffer automatically.
  // Unrecoverable failure.
  if (state_ == State::FAILURE) {
    return false;
  }

  // Handle the message if its for us.
  if (fifo_msg_.destination == FIFOMsg::Destination::SOP && !handle_msg()) {
    FUSB302_FAIL("Failed to handle message");
    return false;
  }

  // // Clear interrupt.
  // if (this->write_register16(REG_INTERRUPT, &interrupt, 1)) {
  //   ESP_LOGE(TAG, "Failed to clear interrupt");
  //   return false;
  // }

  return true;
}

bool FUSB302::read_fifo() {
  uint8_t rx_token;
  if (!this->read_byte(REG_FIFOS, &rx_token)) {
    ESP_LOGE(TAG, "Failed to read FIFO");
    return false;
  }

  uint16_t header;
  if (this->read_register(REG_FIFOS, (uint8_t *) &header, 2)) {
    ESP_LOGE(TAG, "Failed to read header");
    return false;
  }
  // ESP_LOGD(TAG, "Header: 0x%04X", header);
  fifo_msg_.header = header;
  fifo_msg_.n_objs = (header >> 12) & 0x07;
  fifo_msg_.msg_type = header & 0xf;

  if (fifo_msg_.n_objs > FUSB302_MAX_PDOS) {
    ESP_LOGE(TAG, "Too many objects in message of type %d: %d", fifo_msg_.msg_type, fifo_msg_.n_objs);
    return false;
  }

  // Read objects.
  for (uint8_t i = 0; i < fifo_msg_.n_objs; i++) {
    if (this->read_register(REG_FIFOS, (uint8_t *) fifo_msg_.objs + 4 * i, 4)) {
      ESP_LOGE(TAG, "Failed to read object %d", i);
      return false;
    }
  }

  uint32_t crc;
  // Discard CRC.
  if (this->read_register(REG_FIFOS, (uint8_t *) &crc, sizeof(crc))) {
    ESP_LOGE(TAG, "Failed to read CRC");
    return false;
  }

  fifo_msg_.destination = ((rx_token >> 4) & 0x0e) == 0x0e ? FIFOMsg::Destination::SOP : FIFOMsg::Destination::UNKNOWN;

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
      ESP_LOGD(TAG, "Accept message received");
      enter_state(State::TRANSITION_SINK);
    } else if (fifo_msg_.msg_type == 0x06) {  // PS_RDY.
      ESP_LOGD(TAG, "PS_RDY message received");

      // if (state_ == State::REQUESTED_SAFE_5V) {
      //   // We've requested Safe5V and received a PS_RDY message it means we didn't have a suitable PDO. Enter failure
      //   ESP_LOGE(TAG,
      //            "No compatible PDO found for voltage: %u mV; current: %u mA. Requested the safe 5V so we don't lose
      //            " "power altogether. The available Power Delivery Objects are: ", power_requirement_.voltage_mv,
      //            power_requirement_.current_ma);
      //   for (const auto &pdo : pdos_) {
      //     log_pdo(pdo);
      //   }
      //   return false;
      // }
      enter_state(State::READY);
    } else {
      ESP_LOGD(TAG, "Unhandled command message type: 0x%02X", fifo_msg_.msg_type);
    }
  } else {
    // Source_Capabilities.
    if (fifo_msg_.msg_type == 0x01) {
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
      ESP_LOGD(TAG, "Requested PDO: ");
      log_pdo(pdos_[*selected_pdo_idx_]);
    } else {
      ESP_LOGD(TAG, "Unhandled data message type: 0x%02X", fifo_msg_.msg_type);
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
    ESP_LOGE(TAG, "Unsupported PDO type: %d", pdo.type);
    return false;
  }

  if (!this->send_msg(sizeof(request), (uint8_t *) &request)) {
    ESP_LOGE(TAG, "Failed to send PD request");
    return false;
  }
  return true;
}

bool FUSB302::send_msg(uint8_t len, uint8_t *data) {
  static uint8_t msg_id = 0;

  // Truncate to 3 bits (same as % 8).
  msg_id &= 0x7;

  const uint8_t sop[5] = {0x12, 0x12, 0x12, 0x13, 0x80 | (len + static_cast<uint8_t>(2))};
  const uint8_t eop[4] = {0xff, 0x14, 0xfe, 0xa1};

  uint16_t header = 0;
  // Number of objects -- 1.
  header |= (0x1 << 12);
  // Spec revision -- 2.0.
  // header |= (0x1 << 6);
  // Spec revision -- 3.0 (worked with PPS for all adapters).
  // header |= (0x1 << 7);
  // Use same spec revision as the source.
  header |= (pd_spec_ << 6);
  // Message type -- Request.
  header |= (0x1 << 1);
  // Message ID.
  header |= ((msg_id++) << 9);

  // TODO: make this slightly less horrible.
  uint8_t buff[32];
  memcpy(buff, sop, sizeof(sop));
  memcpy(buff + sizeof(sop), (uint8_t *) &header, sizeof(header));
  memcpy(buff + sizeof(sop) + sizeof(header), data, len);
  memcpy(buff + sizeof(sop) + sizeof(header) + len, eop, sizeof(eop));

  if (this->write_register(REG_FIFOS, buff, sizeof(sop) + sizeof(header) + len + sizeof(eop))) {
    ESP_LOGE(TAG, "Failed to write eop to FIFO");
    return false;
  }
  // ESP_LOGD(TAG, "Sent message");
  return true;
}

void FUSB302::maybe_rerequest_pps_pdo() {
  ESP_LOGD(TAG, "Maybe rerequesting PPS PDO. State is: %d", state_);
  if (state_ == State::READY && selected_pdo_idx_.has_value() &&
      pdos_[*selected_pdo_idx_].type == PDO::Type::AUGMENTED &&
      pdos_[*selected_pdo_idx_].augmented.type == PDO::Augmented::Type::SPR_PPS) {
    this->request_pdo();

    // Schedule a new PPS timer.
    this->set_timeout(kPPSTimerName, kPPSTimerIntervalMs, [this]() { this->maybe_rerequest_pps_pdo(); });
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
          if (this->write_byte(REG_CONTROL3, 0b1 << 6)) {
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

      on_pd_negotiation_success_callback_.call(/*success=*/true);

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

}  // namespace fusb302
}  // namespace esphome
