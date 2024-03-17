#include "esphome/components/fusb302/fusb302.h"

#include "esphome/core/log.h"
#include "esphome/components/fusb302/pdo.h"
#include "esphome/components/fusb302/regs.h"

// TODO: Use ESPHome's built-in bit manipulation functions.
// #define HAS_BITS(v, b, n) (((v) >> (b)) & ((1 << (n)) - 1))
// #define HAS_BIT(v, b) (HAS_BITS(v, b, 1))
// #define SWAP16(v) ((((v) >> 8) & 0xff) | (((v) & 0xff) << 8))

namespace esphome {
namespace fusb302 {

static const char *TAG = "fusb302.component";

namespace {

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

}  // namespace

void FUSB302::setup() {
  // Reset.
  if (!this->write_byte(REG_RESET, 0x01)) {
    ESP_LOGE(TAG, "Failed to write to RESET");
    return;
  }

  // delay(10);

  uint8_t device_id;
  if (this->read_register(REG_DEVICE_ID, (uint8_t *) &device_id, 1, false)) {
    ESP_LOGE(TAG, "Failed to read device id");
  } else {
    ESP_LOGV(TAG, "Device id: 0x%04X", device_id);
  }
  // state_ = State::REQUESTED_CAPS;

  // Write to POWER.
  if (!this->write_byte(REG_POWER, 0x0f)) {
    ESP_LOGE(TAG, "Failed to write to POWER");
    return;
  }

  // Enable all interrupts.
  if (!this->write_byte(REG_MASK1, 0x00)) {
    ESP_LOGE(TAG, "Failed to write to MASK1");
    return;
  }
  if (!this->write_byte(REG_MASKA, 0x00)) {
    ESP_LOGE(TAG, "Failed to write to MASKA");
    return;
  }
  if (!this->write_byte(REG_MASKB, 0x00)) {
    ESP_LOGE(TAG, "Failed to write to MASKB");
    return;
  }
  if (!this->write_byte(REG_CONTROL0, 0b11 << 2)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL0");
    return;
  }

  // Enable packet retry.
  if (!this->write_byte(REG_CONTROL3, 0x07)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL3");
    return;
  }

  if (!this->write_byte(REG_CONTROL2, 0x00)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL2");
    return;
  }

  // Clear RX fifo.
  if (!this->write_byte(REG_CONTROL1, 0x04)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL1");
    return;
  }

  // We know by design that we're connected to CC1. For a general approach this has to be detected.

  // Enable CC1 measuring circuit.
  if (!this->write_byte(REG_SWITCHES0, 0x07)) {
    ESP_LOGE(TAG, "Failed to write to SWITCHES0");
    return;
  }
  // Disable pull down?
  // if (!this->write_byte(REG_SWITCHES0, 0x00)) {
  //   ESP_LOGE(TAG, "Failed to write to SWITCHES0");
  //   return;
  // }

  // Flush the TX FIFO.
  if (!this->write_byte(REG_CONTROL0, 0x44)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL0");
    return;
  }
  // Flush the RX FIFO.
  if (!this->write_byte(REG_CONTROL1, 0x1 << 2)) {
    ESP_LOGE(TAG, "Failed to write to CONTROL1");
    return;
  }
  // Reset PD logic.
  if (!this->write_byte(REG_RESET, 0x02)) {
    ESP_LOGE(TAG, "Failed to write to RESET");
    return;
  }

  // Enable auto GoodCRC response and TXCC1, plus keep the revision 2.0 of the GoodCRC ack packet.
  if (!this->write_byte(REG_SWITCHES1, (1 << 0) | (1 << 2) | (1 << 5))) {
    ESP_LOGE(TAG, "Failed to write to SWITCHES1");
    return;
  }
}

void FUSB302::loop() {
  if (state_ == State::FAILURE) {
    return;
  }

  if (!this->process_interrupt()) {
    ESP_LOGE(TAG, "Failed to process interrupt -- assuming failure");
    state_ = State::FAILURE;
    on_pd_negotiation_failure_callback_.call(/*success=*/false);
    return;
  }
}

void FUSB302::update() {}

void FUSB302::dump_config() {
  ESP_LOGCONFIG(TAG, "fusb302");
  LOG_UPDATE_INTERVAL(this);
}

// Interrupt callback.
void FUSB302::ISR(FUSB302 *instance) { instance->interrupt_pending_ = true; }

bool FUSB302::process_interrupt() {
  // Read interrupt.
  uint8_t interrupt;
  if (this->read_register(REG_INTERRUPT, &interrupt, 1)) {
    ESP_LOGE(TAG, "Failed to read interrupt");
    return false;
  }
  // ESP_LOGD(TAG, "Interrupt: 0x%02X", interrupt);

  uint8_t status0, status1;
  if (this->read_register(REG_STATUS0, &status0, 1)) {
    ESP_LOGE(TAG, "Failed to read status0");
    return false;
  }
  if (this->read_register(REG_STATUS1, &status1, 1)) {
    ESP_LOGE(TAG, "Failed to read status1");
    return false;
  }
  // ESP_LOGD(TAG, "Status0: 0x%02X, Status1: 0x%02X", status0, status1);

  // if (state_ == State::RECEIVED_CAPS) {
  //   // Request a PDO.
  //   return this->request_pdo(1);
  // }

  // Is there RX data in the buffer?
  if ((status1 & (1 << 5)) == 0) {
    // ESP_LOGD(TAG, "There is RX data in buffer");
    this->read_fifo();
  }

  // ESP_LOGD(TAG, "RX data in buffer");

  // if (!interrupt) {
  //   // ESP_LOGE(TAG, "Interrupt is not actually set");
  //   return false;
  // }

  // // Clear interrupt.
  // if (this->write_register16(REG_INTERRUPT, &interrupt, 1)) {
  //   ESP_LOGE(TAG, "Failed to clear interrupt");
  //   return false;
  // }

  // interrupt_pending_ = false;
  return true;
}

bool FUSB302::read_fifo() {
  uint8_t rx_token;
  if (!this->read_byte(REG_FIFOS, &rx_token)) {
    ESP_LOGE(TAG, "Failed to read FIFO");
    return false;
  }

  // We only care for SOP messages.
  if (((rx_token >> 4) & 0x0e) == 0x0e) {
    // ESP_LOGD(TAG, "SOP message. RX token: 0x%02X", rx_token);
  } else {
    // ESP_LOGD(TAG, "Not a SOP message. RX token: 0x%02X", rx_token);
  }

  uint16_t header;
  // uint8_t header[2];
  if (this->read_register(REG_FIFOS, (uint8_t *) &header, 2)) {
    ESP_LOGE(TAG, "Failed to read header");
    return false;
  }
  ESP_LOGD(TAG, "Header: 0x%04X", header);

  uint8_t n_objects = (header >> (12 - 0)) & 0x07;
  ESP_LOGD(TAG, "Number of objects: %d", n_objects);

  uint8_t msg_type = header & 0xf;

  // Read objects.
  uint32_t objs[FUSB302_MAX_PDOS];
  for (uint8_t i = 0; i < n_objects; i++) {
    if (this->read_register(REG_FIFOS, (uint8_t *) objs + 4 * i, 4)) {
      ESP_LOGE(TAG, "Failed to read object %d", i);
      return false;
    }
    // ESP_LOGD(TAG, "Object %d: 0x%08X", i, objs[i]);
    // PDO pdo = parse_pdo(objs[i]);
    // log_pdo(pdo);
  }

  uint32_t crc;
  // Discard CRC.
  if (this->read_register(REG_FIFOS, (uint8_t *) &crc, sizeof(crc))) {
    ESP_LOGE(TAG, "Failed to read CRC");
    return false;
  }
  // ESP_LOGD(TAG, "CRC: 0x%08X", crc);

  return handle_msg(msg_type, n_objects, objs);
}

bool FUSB302::handle_msg(uint8_t msg_type, uint8_t n_objects, uint32_t *objs) {
  if (n_objects == 0) {
    // ESP_LOGD(TAG, "No objects in message -- command message. Type: 0x%02X", msg_type);
    if (msg_type == 0x03) {  // Accept.
      ESP_LOGD(TAG, "Accept message received");
    } else if (msg_type == 0x06) {  // PS_RDY.
      ESP_LOGD(TAG, "PS_RDY message received");
      state_ = State::READY;
      // Call callbacks here.
      on_pd_negotiation_success_callback_.call(/*success=*/true);
    } else {
      ESP_LOGD(TAG, "Unhandled command message type: 0x%02X", msg_type);
    }
  } else {
    // Source_Capabilities.
    if (msg_type == 0x01) {
      state_ = State::RECEIVED_CAPS;
      // We have to be fast to send this response. Otherwise the power supply will hard reset.
      request_pdo(1);
      state_ = State::REQUESTED_PDO;
      ESP_LOGD(TAG, "Source capabilities received and requested. New state: %d", static_cast<int>(state_));
    } else {
      ESP_LOGD(TAG, "Unhandled data message type: 0x%02X", msg_type);
    }
  }
  return true;
}
bool FUSB302::request_pdo(uint8_t pdo_idx) {
  // ESP_LOGD(TAG, "Requesting fixed PDO with index %d", pdo_idx);

  uint32_t request = 0;

  // Object position -- index + 1.
  request |= (((pdo_idx + 1) & 0x07) << 28);

  // USB communications capability.
  request |= (0x1 << 25);

  // No USB suspend.
  // request |= (0x1 << 24);

  // Unchunked message supported.
  request |= (0x1 << 23);

  uint32_t current_10ma = 150;
  request |= (current_10ma << 10);
  request |= current_10ma;

  // ESP_LOGV(TAG, "Will send RDO: 0x%08X", request);
  // dump_rdo(&request, pdos_);

  // Invert.
  // request = byteswap(request);
  // if (this->write_register16(REG_REQUEST, (uint8_t *) &request, sizeof(request))) {
  //   // TODO: fatal.
  //   ESP_LOGE(TAG, "Failed to write PD request");
  //   return false;
  // }

  if (!this->send_msg(sizeof(request), (uint8_t *) &request)) {
    ESP_LOGE(TAG, "Failed to send PD request");
    return false;
  }

  state_ = State::REQUESTED_PDO;
  ESP_LOGD(TAG, "Requested PDO");
  return true;
}

bool FUSB302::send_msg(size_t len, uint8_t *data) {
  const uint8_t sop[5] = {0x12, 0x12, 0x12, 0x13, 0x80 | (((int) len) + 2)};
  const uint8_t eop[4] = {0xff, 0x14, 0xfe, 0xa1};

  uint16_t header = 0;
  // Number of objects -- 1.
  header |= 0x1 << 12;
  // Spec revision -- 2.0.
  header |= 0x1 << 6;
  // Message type -- Request.
  header |= 0x1 << 1;

  uint8_t buff[32];
  memcpy(buff, sop, sizeof(sop));
  memcpy(buff + sizeof(sop), (uint8_t *) &header, sizeof(header));
  memcpy(buff + sizeof(sop) + sizeof(header), data, len);
  memcpy(buff + sizeof(sop) + sizeof(header) + len, eop, sizeof(eop));

  if (this->write_register(REG_FIFOS, buff, sizeof(sop) + sizeof(header) + len + sizeof(eop))) {
    ESP_LOGE(TAG, "Failed to write eop to FIFO");
    return false;
  }
  ESP_LOGD(TAG, "Sent message");
  return true;
}

}  // namespace fusb302
}  // namespace esphome
