#include "esphome/components/fusb302/fusb302.h"

#include "esphome/core/log.h"
#include "esphome/components/fusb302/pdo.h"
#include "esphome/components/fusb302/regs.h"
#include "esphome/components/fusb302/consts.h"
#include "esphome/components/fusb302/crc32.h"
#include "esphome/core/hal.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define FUSB302_FAIL(str, ...) \
  do { \
    ESP_LOGE(TAG, str, ##__VA_ARGS__); \
    enter_state(State::FAILURE); \
  } while (0)

namespace esphome {
namespace fusb302 {

static const char *TAG = "fusb302.component";

namespace {

// Timer names.
constexpr char kWaitForCapsTimerName[] = "wait_for_caps";
constexpr char kPPSTimerName[] = "pps_timer";
constexpr char kEPRKeepaliveTimerName[] = "epr_keepalive_timer";
constexpr char kSoftResetWatchdogTimerName[] = "soft_watchdog";

// Make a Request Data Object for a fixed PDO.
uint32_t make_fixed_rdo(uint8_t pdo_idx, uint16_t max_current_ma) {
  uint32_t rdo = 0;
  rdo |= ((pdo_idx + 1) << kFixedRDOObjectPositionShift);
  rdo |= kFixedRDOUSBCap;
  rdo |= kFixedRDONoUSBSuspend;
  rdo |= kFixedRDOEPRCapable;
  rdo |= (max_current_ma / 10) << kFixedRDOOperatingCurrentShift;
  rdo |= (max_current_ma / 10) << kFixedRDOMaxCurrentShift;
  return rdo;
}

// Make a Request Data Object for a PPS PDO.
uint32_t make_pps_rdo(uint8_t pdo_idx, uint16_t voltage_mv, uint16_t current_ma) {
  uint32_t rdo = 0;
  rdo |= ((pdo_idx + 1) << kPPSRDOObjectPositionShift);
  rdo |= (voltage_mv / 20) << kPPSRDOOutputVoltageShift;
  rdo |= (current_ma / 50) << kPPSRDOOutputCurrentShift;
  return rdo;
}

}  // namespace

void FUSB302::dump_config() {
  ESP_LOGCONFIG(TAG, "fusb302");
  LOG_UPDATE_INTERVAL(this);
}

bool FUSB302::measure_cc_pin(uint8_t cc_pin, uint8_t *voltage_out) {
  // Enable CCx measuring circuit.
  if (!this->write_byte(REG_SWITCHES0, REG_SWITCHES0_PULLDOWN_CC1 | REG_SWITCHES0_PULLDOWN_CC2 | (cc_pin << 2))) {
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

void FUSB302::set_power_requirement(uint16_t voltage_mv, uint16_t current_ma) {
  // Sanity check against spec and FUSB302 max ratings.
  if (voltage_mv < 3300 || voltage_mv > 28000) {
    FUSB302_FAIL("Voltage must be between 3300 and 22000 mV. Got: %d mV", voltage_mv);
    return;
  } else if (current_ma > 5000) {
    FUSB302_FAIL("Current must be between 0 and 5000 mA. Got: %d mA", current_ma);
    return;
  }
  this->power_requirement_.voltage_mv = voltage_mv;
  this->power_requirement_.current_ma = current_ma;
}

void FUSB302::setup() {
  // Set up the interrupt pin if it's set.
  if (int_pin_ != nullptr) {
    ESP_LOGD(TAG, "Setting up interrupt pin");
    int_pin_->setup();
    int_pin_->attach_interrupt(FUSB302::ISR, this, gpio::INTERRUPT_FALLING_EDGE);
  }

  if (this->start_power_negotiation_on_boot_) {
    this->start_power_negotiation();
  }
}

void FUSB302::start_power_negotiation() {
  if (!this->write_byte(REG_RESET, REG_RESET_SW_RESET | REG_RESET_PD_RESET)) {
    FUSB302_FAIL("Failed to write to RESET");
    return;
  }

  if (!this->write_byte(REG_POWER, REG_POWER_POWER_ALL)) {
    FUSB302_FAIL("Failed to write to POWER");
    return;
  }

  uint8_t device_id;
  if (this->read_register(REG_DEVICE_ID, (uint8_t *) &device_id, 1, false)) {
    FUSB302_FAIL("Failed to read device id");
    return;
  } else {
    ESP_LOGV(TAG, "Device id: 0x%04X", device_id);
  }

  if (!this->write_byte(REG_MASK1, REG_MASK1_MASK_ALL)) {
    FUSB302_FAIL("Failed to write to MASK1");
    return;
  }

  if (!this->write_byte(REG_MASKA, REG_MASKA_MASK_NONE)) {
    FUSB302_FAIL("Failed to write to MASKA");
    return;
  }
  if (!this->write_byte(REG_MASKB, REG_MASKB_MASK_NONE)) {
    FUSB302_FAIL("Failed to write to MASKB");
    return;
  }

  // if (!this->write_byte(REG_CONTROL0, REG_CONTROL0_HOST_CUR_HIGH)) {
  //   ESP_LOGE(TAG, "Failed to write to CONTROL0");
  //   FUSB302_FAIL("Failed to write to CONTROL0");
  //   return;
  // }

  // Enable packet retry.
  if (!this->write_byte(REG_CONTROL3, REG_CONTROL3_AUTO_RETRY | REG_CONTROL3_N_RETRIES_3)) {
    FUSB302_FAIL("Failed to write to CONTROL3");
    return;
  }

  if (!this->write_byte(REG_CONTROL2, REG_CONTROL2_DO_NOT_USE)) {
    FUSB302_FAIL("Failed to write to CONTROL2");
    return;
  }

  // // Clear RX fifo.
  // if (!this->write_byte(REG_CONTROL1, REG_CONTROL1_RX_FLUSH)) {
  //   FUSB302_FAIL("Failed to write to CONTROL1");
  //   return;
  // }

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

  const uint8_t cc_pin = cc1_voltage > cc2_voltage ? 1 : 2;
  if (!this->write_byte(REG_SWITCHES0, REG_SWITCHES0_PULLDOWN_CC1 | REG_SWITCHES0_PULLDOWN_CC2 |
                                           (cc_pin == 1 ? REG_SWITCHES0_MEAS_CC1 : REG_SWITCHES0_MEAS_CC2))) {
    FUSB302_FAIL("Failed to write to SWITCHES0");
    return;
  }

  // Enable auto GoodCRC response and TXCCx, plus keep the revision 2.0 of the GoodCRC ack packet.
  if (!this->write_byte(REG_SWITCHES1, (cc_pin == 1 ? REG_SWITCHES1_TXCC1 : REG_SWITCHES1_TXCC2) |
                                           REG_SWITCHES1_AUTO_CRC | REG_SWITCHES1_SPEC2)) {
    FUSB302_FAIL("Failed to write to SWITCHES1");
    return;
  }

  // Flush the TX FIFO.
  if (!this->write_byte(REG_CONTROL0, REG_CONTROL0_HOST_CUR_HIGH | REG_CONTROL0_TX_FLUSH)) {
    FUSB302_FAIL("Failed to write to CONTROL0");
    return;
  }
  // Flush the RX FIFO.
  if (!this->write_byte(REG_CONTROL1, REG_CONTROL1_RX_FLUSH)) {
    FUSB302_FAIL("Failed to write to CONTROL1");
    return;
  }

  // Reset PD logic.
  if (!this->write_byte(REG_RESET, REG_RESET_PD_RESET)) {
    FUSB302_FAIL("Failed to write to RESET");
    return;
  }

  // We have to be as fast as we can during the negotiation phase, so we'll use a high frequency loop. We may disable
  // it once the negotiation is complete.
  high_freq_loop_req_.start();

  enter_state(State::WAIT_FOR_CAPABILITIES);

  power_negotiation_started_ = true;
}

void FUSB302::loop() {
  LockGuard lock(mutex_);

  if (state_ == State::FAILURE) {
    return;
  }

  // Continuously poll if no interrupt pin is set, or if an interrupt is pending.
  if ((this->int_pin_ == nullptr || this->interrupt_pending_) && power_negotiation_started_) {
    this->interrupt_pending_ = false;
    this->process_interrupt();
    ESP_LOGD(TAG, "Processed interrupt");
  }
}

void FUSB302::update() {}

// Interrupt callback.
void FUSB302::ISR(FUSB302 *instance) { instance->interrupt_pending_ = true; }

bool FUSB302::process_interrupt() {
  ESP_LOGD(TAG, "Processing interrupt");

  // Read all interrupt registers in one go.
  volatile uint8_t buf[7];
  i2c::ErrorCode err;
  if ((err = this->read_register(REG_STATUS0A, (uint8_t *) &buf, sizeof(buf), true))) {
    ESP_LOGE(TAG, "Failed to read reg. Error: %d", err);
    return false;
  }

  while (this->has_fifo_msg()) {
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
  if (!this->read_byte(REG_STATUS1, &status1)) {
    ESP_LOGE(TAG, "Failed to read status1");
    return false;
  }
  return (status1 & (1 << 5)) == 0;
}

bool FUSB302::read_fifo() {
  // RX token (1) + header (2) + extended_header (2) +  4 * MAX_PDOS + CRC (4).
  uint8_t buf[sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint16_t) + 4 * kMaxPDOS + sizeof(uint32_t)];

  // Read RX token and header.
  if (this->read_register(REG_FIFOS, buf, sizeof(uint8_t) + sizeof(uint16_t))) {
    ESP_LOGE(TAG, "Failed to read FIFO rx token and header");
    return false;
  }

  uint8_t rx_token = buf[0];
  fifo_msg_.header = buf[2] << 8 | buf[1];
  fifo_msg_.n_objs = (fifo_msg_.header >> 12) & 0x07;
  fifo_msg_.msg_type = fifo_msg_.header & 0x1f;
  fifo_msg_.extended = (fifo_msg_.header >> 15) & 0x1;

  if (fifo_msg_.n_objs > kMaxPDOS) {
    ESP_LOGE(TAG, "Too many objects in message of type %d: %d", fifo_msg_.msg_type, fifo_msg_.n_objs);
    return false;
  }

  // Data starts after RX token (1 byte) + header (2 bytes).
  uint8_t *data_buf = buf + sizeof(rx_token) + sizeof(fifo_msg_.header);

  // Read data objects + CRC.
  if (this->read_register(REG_FIFOS, data_buf, 4 * fifo_msg_.n_objs + 4)) {
    ESP_LOGE(TAG, "Failed to read objects");
    return false;
  }

  // Write into fifo_msg_.
  for (uint8_t i = 0; i < fifo_msg_.n_objs; i++) {
    fifo_msg_.objs[i] =
        data_buf[i * 4 + 3] << 24 | data_buf[i * 4 + 2] << 16 | data_buf[i * 4 + 1] << 8 | data_buf[i * 4];
  }

  fifo_msg_.destination = ((rx_token >> 4) & 0x0e) == 0x0e ? FIFOMsg::Destination::SOP : FIFOMsg::Destination::UNKNOWN;

  const uint8_t *crc_buf = data_buf + 4 * fifo_msg_.n_objs;
  uint32_t crc = crc_buf[3] << 24 | crc_buf[2] << 16 | crc_buf[1] << 8 | crc_buf[0];
  uint32_t crc_calc = crc32(buf + 1, crc_buf - (buf + 1));

  if (crc != crc_calc) {
    ESP_LOGE(TAG, "CRC mismatch! 0x%08X != 0x%08X", crc, crc_calc);
    return false;
  }

  return true;
}

bool FUSB302::handle_msg() {
  if (fifo_msg_.extended) {
    return handle_extended_msg();
  }

  if (fifo_msg_.n_objs == 0) {
    if (fifo_msg_.msg_type == kCtrlMsgTypeGoodCRC) {
      return true;
    } else if (fifo_msg_.msg_type == kCtrlMsgTypeAccept) {
      ESP_LOGW(TAG, "Accept message received");
      enter_state(State::TRANSITION_SINK);
    } else if (fifo_msg_.msg_type == kCtrlMsgTypePSReady) {
      ESP_LOGW(TAG, "PS_RDY message received");
      enter_state(State::READY);
    } else if (fifo_msg_.msg_type == kCtrlMsgTypeSoftReset) {
      // Send an Accept message right away. If things go well, the source will send a Source_Capabilities message
      // shortly after.
      if (!this->send_msg(kCtrlMsgTypeAccept, 0, nullptr)) {
        ESP_LOGE(TAG, "Failed to send Accept in response to Soft_Reset");
        return false;
      }
      ESP_LOGW(TAG, "Soft_Reset received, replied with Accept");
    } else {
      ESP_LOGW(TAG, "Unhandled command message type: 0x%02X", fifo_msg_.msg_type);
    }
  } else {
    if (fifo_msg_.msg_type == kDataMsgTypeSourceCapabilities) {
      pd_spec_ = (fifo_msg_.header >> 6) & 0x3;
      if (!this->parse_pdos(fifo_msg_.n_objs, fifo_msg_.objs)) {
        ESP_LOGE(TAG, "Failed to parse PDOS");
        return false;
      }
      enter_state(State::EVALUATE_CAPABILITY);
    } else if (fifo_msg_.msg_type == kDataMsgTypeEPRMode) {
      uint8_t action = (fifo_msg_.objs[0] >> 24) & 0xff;
      if (action == 0x2) {  // Enter Acknowledged.
        ESP_LOGW(TAG, "Received EPR_Mode: Enter Acknowledged.");
      } else if (action == 0x3) {  // Enter Succeeded.
        epr_mode_ = true;
        ESP_LOGW(TAG, "Received EPR_Mode: Enter Succeeded.");
      } else if (action == 0x4) {  // Enter Failed.
        uint8_t cause = (fifo_msg_.objs[0] >> 16) & 0xff;
        ESP_LOGW(TAG, "Received EPR_Mode: Enter Failed. Cause: %d", cause);
      } else {
        ESP_LOGE(TAG, "Received unexpected EPR_Mode message. Action: %d. Header: 0x%04X", action, fifo_msg_.header);
      }
    } else {
      ESP_LOGW(TAG, "Unhandled data message type: 0x%02X", fifo_msg_.msg_type);
    }
  }
  return true;
}

bool FUSB302::handle_extended_msg() {
  uint16_t ext_header = fifo_msg_.objs[0] & 0xffff;
  uint16_t data_size = ext_header & 0x1ff;
  bool chunked = (ext_header >> 15) & 0x1;
  uint8_t chunk_number = (ext_header >> 11) & 0x0f;

  // Data starts after the extended header (2 bytes).
  const uint8_t *data = (const uint8_t *) fifo_msg_.objs + sizeof(ext_header);
  // Discount the header (2 bytes). The "padding" in the spec in this case is actually half of the PDO in the next
  // chunk.
  const size_t actual_len = fifo_msg_.n_objs * 4 - sizeof(uint16_t);

  ESP_LOGD(TAG, "header.n_objs: %d, Received chunk %d chunked message. Data size: %d, received bytes: %d",
           fifo_msg_.n_objs, chunk_number, data_size, actual_len);

  if (chunk_number == 0) {
    chunked_buffer_.total_len = data_size;
    memcpy(chunked_buffer_.data, data, actual_len);
    chunked_buffer_.current_len = actual_len;
  } else {
    memcpy(chunked_buffer_.data + chunked_buffer_.current_len, data, actual_len);
    chunked_buffer_.current_len += actual_len;
  }

  // Are we done?
  ESP_LOGD(TAG, "Chunked buffer: %d/%d", chunked_buffer_.current_len, chunked_buffer_.total_len);
  if (chunked_buffer_.current_len != chunked_buffer_.total_len) {
    // Request next chunk.
    return this->send_chunk_request(chunk_number);
  }

  // We're done collating the chunks & have a full message.
  uint8_t n_objs = chunked_buffer_.total_len / sizeof(uint32_t);

  // Control message.
  if (n_objs == 0) {
    if (fifo_msg_.msg_type == kExtMsgTypeExtendedControl) {
      ESP_LOGD(TAG, "Received EPR_KeepAlive_Ack");
    } else {
      ESP_LOGW(TAG, "Unhandled extended control message type: 0x%02X", fifo_msg_.msg_type);
    }
  } else {  // Data message.
    if (fifo_msg_.msg_type == kExtMsgTypeEPRSourceCapabilities) {
      // Extract the PD spec revision.
      pd_spec_ = (fifo_msg_.header >> 6) & 0x3;

      uint8_t n_pdos = chunked_buffer_.total_len / sizeof(uint32_t);
      if (!this->parse_pdos(n_pdos, (uint32_t *) chunked_buffer_.data)) {
        ESP_LOGE(TAG, "Failed to parse PDOS");
        return false;
      }
      enter_state(State::EVALUATE_CAPABILITY);
      return true;
    } else {
      ESP_LOGW(TAG, "Unhandled extended data message type: 0x%02X", fifo_msg_.msg_type);
    }
  }

  // Reset the buffer.
  chunked_buffer_.total_len = 0;
  chunked_buffer_.current_len = 0;

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
    }
  }

  return true;
}

bool FUSB302::evaluate_capabilities() {
  // PDOs must already be parsed.
  selected_pdo_idx_.reset();
  for (uint8_t i = 0; i < pdos_.size(); i++) {
    const PDO &pdo = pdos_[i];
    // Select the first compatible PDO.
    if (pdo.parsed && is_pdo_compatible(pdo, power_requirement_)) {
      selected_pdo_idx_ = i;
      break;
    }
  }
  if (!selected_pdo_idx_.has_value()) {
    // If we don't find a compatible PDO, we'll just select the first one, which is guaranteed to be Safe5V as per spec.
    selected_pdo_idx_ = 0;
  }
  if (!this->request_pdo()) {
    ESP_LOGE(TAG, "Failed to request PDO");
    return false;
  }
  enter_state(State::SELECT_CAPABILITY);
  return true;
}

bool FUSB302::request_pdo() {
  const PDO &pdo = pdos_[*selected_pdo_idx_];
  uint32_t request;
  if (pdo.type == PDO::Type::FIXED) {
    request = make_fixed_rdo(*selected_pdo_idx_, power_requirement_.current_ma);
  } else if (pdo.type == PDO::Type::AUGMENTED && pdo.augmented.type == PDO::Augmented::Type::SPR_PPS) {
    request = make_pps_rdo(*selected_pdo_idx_, power_requirement_.voltage_mv, power_requirement_.current_ma);
  } else {
    ESP_LOGE(TAG, "Unsupported selected PDO type: %d", static_cast<int>(pdo.type));
    return false;
  }

  if (!epr_mode_) {
    if (!this->send_msg(kDataMsgTypeRequest, sizeof(request), (uint8_t *) &request)) {
      ESP_LOGE(TAG, "Failed to send PD request");
      return false;
    }
    return true;
  }

  // In EPR mode, we have to send an EPR_Request, even when requesting a SPR PDO.
  uint32_t *pdo0 = ((uint32_t *) chunked_buffer_.data) + selected_pdo_idx_.value_or(0);
  uint8_t buf[sizeof(request) + sizeof(pdo0)];
  memcpy(buf, &request, sizeof(request));
  memcpy(buf + sizeof(request), pdo0, sizeof(*pdo0));
  return this->send_msg(kDataMsgTypeEPRRequest, sizeof(buf), buf);
}

bool FUSB302::send_soft_reset() {
  // Try our best to flush the contents for RX and TX fifo.
  if (!this->write_byte(REG_CONTROL0, REG_CONTROL0_TX_FLUSH)) {
    ESP_LOGE(TAG, "Failed to flush TX FIFO");
    return false;
  }
  if (!this->write_byte(REG_CONTROL1, REG_CONTROL1_RX_FLUSH)) {
    ESP_LOGE(TAG, "Failed to flush TX FIFO");
    return false;
  }
  if (!this->send_msg(kCtrlMsgTypeSoftReset, 0, nullptr)) {
    ESP_LOGE(TAG, "Failed to send soft reset");
    return false;
  }
  return true;
}

bool FUSB302::send_epr_mode_enter() {
  uint32_t eprmdo = 0;
  eprmdo |= (kEPRModeActionEnter << kEPRModeActionShift);
  // EPR sink operational PDP in 1W units.
  uint16_t power = ((power_requirement_.voltage_mv / 1000) * power_requirement_.current_ma) / 1000;
  eprmdo |= (power << kEPRModePowerShift);

  if (!this->send_msg(kDataMsgTypeEPRMode, sizeof(eprmdo), (uint8_t *) &eprmdo)) {
    ESP_LOGE(TAG, "Failed to send EPR_Mode enter message");
    return false;
  }
  return true;
}

bool FUSB302::send_epr_mode_exit() {
  uint32_t eprmdo = 0;
  eprmdo |= (kEPRModeActionExit << kEPRModeActionShift);
  if (!this->send_msg(kDataMsgTypeEPRMode, sizeof(eprmdo), (uint8_t *) &eprmdo)) {
    ESP_LOGE(TAG, "Failed to send EPR_Mode enter message");
    return false;
  }
  return true;
}

bool FUSB302::send_chunk_request(uint8_t chunk_number) {
  uint16_t ext_header = 0;
  ext_header |= kExtHeaderChunked;
  ext_header |= ((chunk_number + 1) << kExtHeaderChunkNumberShift);
  ext_header |= kExtHeaderRequestChunk;

  // Padding.
  uint32_t padded_data = 0;
  padded_data |= ext_header;

  // Assume we only request extra chunks for EPR_Source_Capabilities.
  if (!this->send_msg(kExtMsgTypeEPRSourceCapabilities, sizeof(padded_data), (uint8_t *) &padded_data,
                      /*extended=*/true)) {
    ESP_LOGE(TAG, "Failed to send Chunk Request message");
    return false;
  }
  return true;
}

bool FUSB302::send_msg(uint8_t msg_type, uint8_t len, uint8_t *data, bool extended) {
  static uint8_t msg_id = 0;

  // Truncate to 3 bits.
  msg_id &= 0x7;

  // header (2 bytes) + data (len bytes) + crc (4 bytes).
  const uint8_t last = kTokPACKSYM | (sizeof(uint16_t) + len + sizeof(uint32_t));
  const uint8_t sop[] = {kTokSOP1, kTokSOP1, kTokSOP1, kTokSOP2, last};
  const uint8_t eop[] = {kTokEOP, kTokTXOFF, kTokTXON};

  uint16_t header = 0;
  // Each object is 4 bytes.
  header |= ((len / sizeof(uint32_t)) << kMsgHeaderNumberOfObjsShift);
  header |= (pd_spec_ << kMsgHeaderPDSpecShift);
  header |= msg_type;
  header |= ((msg_id++) << kMsgHeaderMsgIdShift);
  header |= (extended ? kMsgHeaderExtended : 0);

  uint8_t buff[sizeof(sop) + sizeof(header) + kMaxPDOS * sizeof(uint32_t) + sizeof(uint32_t) + sizeof(eop)];
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

  if (this->write_register(REG_FIFOS, buff, pos)) {
    ESP_LOGE(TAG, "Failed to write eop to FIFO");
  }
  return true;
}

void FUSB302::maybe_rerequest_pps_pdo() {
  LockGuard lock(mutex_);

  // Schedule a new soft reset watchdog timer.
  this->set_timeout(kSoftResetWatchdogTimerName, tSoftResetWatchdogIntervalMs, [this]() { FUSB302::Watchdog(this); });

  ESP_LOGW(TAG, "Maybe rerequesting PPS PDO. State is: %d", static_cast<int>(state_));
  if (selected_pdo_idx_.has_value() && pdos_[*selected_pdo_idx_].type == PDO::Type::AUGMENTED &&
      pdos_[*selected_pdo_idx_].augmented.type == PDO::Augmented::Type::SPR_PPS) {
    ESP_LOGW(TAG, "Ok! Re-requesting PPS PDO.");
    this->request_pdo();

    enter_state(State::SELECT_CAPABILITY);

    ESP_LOGW(TAG, "Done re-requesting PDO. Re-scheduling PPS timer");
    // Schedule a new PPS timer.
    this->set_timeout(kPPSTimerName, tPPSTimerIntervalMs, [this]() { this->maybe_rerequest_pps_pdo(); });
  } else {
    ESP_LOGE(TAG, "Not in PPS mode!");
  }
}

void FUSB302::maybe_send_epr_keepalive() {
  ESP_LOGI(TAG, "Maybe sending EPR keepalive. State is: %d", static_cast<int>(state_));
  if (!epr_mode_) {
    ESP_LOGE(TAG, "Not in EPR mode!");
    return;
  }
  ESP_LOGI(TAG, "Sending EPR keepalive.");

  uint16_t ecdb = kExtMsgTypeEPRKeepAlive;
  uint16_t ext_header = sizeof(ecdb) | kExtHeaderChunked;
  uint32_t keepalive = (ecdb << 16) | ext_header;

  if (!this->send_msg(kExtMsgTypeExtendedControl, sizeof(keepalive), (uint8_t *) &keepalive, /*extended=*/true)) {
    ESP_LOGE(TAG, "Failed to send EPR keepalive");
    return;
  }

  // Schedule a new EPR keepalive timer.
  this->set_timeout(kEPRKeepaliveTimerName, tSourceEPRKeepAlive, [this]() { this->maybe_send_epr_keepalive(); });
}

void FUSB302::enter_state(State state) {
  if (state_ == state) {
    return;
  }

  state_ = state;

  switch (state_) {
    case State::WAIT_FOR_CAPABILITIES: {
      // Start the SinkWaitCapTimer.
      this->set_timeout(kWaitForCapsTimerName, tTypeCSinkWaitCap, [this]() {
        ESP_LOGE(TAG, "Timed out waiting for capabilities. This is expected if ESPHome was reset, but the USB-C cable "
                      "remained connected.");
        FUSB302::Watchdog(this);
      });
      break;
    }
    case State::EVALUATE_CAPABILITY: {
      // We received capabilities. We can cancel the SinkWaitCapTimer timer.
      cancel_timeout(kWaitForCapsTimerName);
      // Start our own watchdog. We should we in the READY state in tSoftResetWatchdogIntervalMs.
      this->set_timeout(kSoftResetWatchdogTimerName, tSoftResetWatchdogIntervalMs,
                        [this]() { FUSB302::Watchdog(this); });

      this->evaluate_capabilities();
      break;
    }
    case State::READY: {
      // We can probably go back to usual loop update frequency.
      // high_freq_loop_req_.stop();

      // Do we need to schedule a PPS timer?
      if (selected_pdo_idx_.has_value() && pdos_[*selected_pdo_idx_].type == PDO::Type::AUGMENTED &&
          pdos_[*selected_pdo_idx_].augmented.type == PDO::Augmented::Type::SPR_PPS) {
        this->set_timeout(kPPSTimerName, tPPSTimerIntervalMs, [this]() { this->maybe_rerequest_pps_pdo(); });
      }

      // Do we need to schedule an EPR keepalive timer?
      if (epr_mode_) {
        this->set_timeout(kEPRKeepaliveTimerName, tSourceEPRKeepAlive, [this]() { this->maybe_send_epr_keepalive(); });
      }

      // Is this the voltage we wanted or a fallback Safe5V?;
      if (selected_pdo_idx_.has_value() && is_pdo_compatible(pdos_[*selected_pdo_idx_], power_requirement_)) {
        on_pd_negotiation_success_callback_.call(/*success=*/true);
        if (first_ready_state_) {
          ESP_LOGI(TAG, "Succesfully Negotiated PDO:");
          FUSB302_LOG_PDO(ESP_LOGI, pdos_[*selected_pdo_idx_]);
          ESP_LOGI(TAG, "All available PDOs:");
          for (const auto &pdo : pdos_) {
            FUSB302_LOG_PDO(ESP_LOGI, pdo);
          }
        }
        first_ready_state_ = false;
      } else {
        // We selected the Safe5V, but we can try to go into EPR mode if the source supports it.
        if (selected_pdo_idx_.value_or(-1) == 0 && pdos_[0].type == PDO::Type::FIXED &&
            pdos_[0].fixed.epr_mode_capable && !epr_mode_) {
          ESP_LOGW(TAG, "Selected Safe5V, but EPR mode capable. Trying to go into EPR mode.");
          FUSB302_LOG_PDO(ESP_LOGW, pdos_[0]);
          if (!this->send_epr_mode_enter()) {
            ESP_LOGE(TAG, "Failed to send EPR mode enter message");
          }
          return;

        }
        // Nothing else we can do, the requested power cannot be provided.
        else {
          on_pd_negotiation_failure_callback_.call(/*success=*/false);
          ESP_LOGI(TAG,
                   "No compatible PDO found for voltage: %u mV; current: %u mA. Negotiated the fallback safe 5V. The "
                   "available Power Delivery Objects are: ",
                   power_requirement_.voltage_mv, power_requirement_.current_ma);
          return;
        }
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

void FUSB302::Watchdog(FUSB302 *instance) {
  LockGuard lock(instance->mutex_);

  // If we're in failure, we can't do anything.
  if (instance->state_ == State::FAILURE) {
    return;
  }

  // We will try to fix the issue first, otherwise we will revisit the watchdog.
  instance->set_timeout(kSoftResetWatchdogTimerName, tSoftResetWatchdogIntervalMs,
                        [instance]() { instance->Watchdog(instance); });

  // If interrupt is asserted, something is wrong. Could be caused by an error in clearing the interrupt.
  if (instance->int_pin_ != nullptr && instance->int_pin_->digital_read() == 0) {
    ESP_LOGE(TAG, "Interrupt is asserted. Something is wrong.");
    // This will re-clear the interrupt.
    instance->process_interrupt();
    return;
  }

  // We're good if we're in READY state.
  if (instance->state_ == State::READY) {
    ESP_LOGW(TAG, "Watchdog expired, but we're in READY state. Not doing anything.");
    instance->cancel_timeout(kSoftResetWatchdogTimerName);
    return;
  }

  // Send a soft reset.
  ESP_LOGE(TAG, "Watchdog expected READY state (and we're at %d) -- sending a soft reset.",
           static_cast<int>(instance->state_));

  if (instance->send_soft_reset()) {
    ESP_LOGE(TAG, "Soft reset sent.");
    return;
  }

  ESP_LOGE(TAG, "Error sending soft reset.");
}

}  // namespace fusb302
}  // namespace esphome
