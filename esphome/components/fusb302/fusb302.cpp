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
  this->process_interrupt();
}

void FUSB302::update() {
  // if (state_ == State::FAILURE) {
  //   return;
  // }

  // float vbus_voltage = this->get_vbus_voltage();
  // if (vbus_voltage > 0.0f && this->vbus_voltage_sensor_ != nullptr) {
  //   this->vbus_voltage_sensor_->publish_state(vbus_voltage);
  // }
}

void FUSB302::dump_config() {
  ESP_LOGCONFIG(TAG, "fusb302");
  LOG_UPDATE_INTERVAL(this);
}

// float FUSB302::get_vbus_voltage() {
//   uint8_t bus_voltage_dv;
//   if (this->read_register16(REG_BUS_VOLTAGE, &bus_voltage_dv, 1)) {
//     ESP_LOGE(TAG, "Failed to read bus voltage");
//     return 0.0f;
//   }
//   float bus_voltage = bus_voltage_dv * 0.1f;
//   ESP_LOGD(TAG, "Bus voltage: %.2f", bus_voltage);
//   return bus_voltage;
// }

// PDO FUSB302::get_current_pdo() {
//   // TODO: uint32_t.
//   uint8_t pdo_bytes[4];
//   if (this->read_register16(REG_CURRENT_PDO, pdo_bytes, sizeof(pdo_bytes))) {
//     ESP_LOGE(TAG, "Failed to read current PDO");
//     return PDO{};
//   }

//   ESP_LOGD(TAG, "PDO: %02X %02X %02X %02X\n", pdo_bytes[0], pdo_bytes[1], pdo_bytes[2], pdo_bytes[3]);

//   // Bytes are received in little endian.
//   uint32_t pdo_data = pdo_bytes[0] | (pdo_bytes[1] << 8) | (pdo_bytes[2] << 16) | (pdo_bytes[3] << 24);

//   PDO pdo = parse_pdo(pdo_data);
//   if (!pdo.parsed) {
//     ESP_LOGE(TAG, "Failed to parse PDO");
//     return PDO{};
//   }
//   return pdo;
// }

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

// bool FUSB302::handle_event_status(uint32_t event_status) { return true; }

// bool FUSB302::handle_pd_response(uint32_t pd_response) {
//   ESP_LOGD(TAG, "PD response: 0x%08X -- %s", pd_response, pd_response & (1 << 7) ? "ASYNC" : "CMD");

//   // This seems weird. From the datasheet, we should do & 0x7f, but that doesn't work as some response codes are
//   larger
//   // than 0x7f. The response type is likely included in the code, making it 0xff.
//   ResponseCode code = static_cast<ResponseCode>(pd_response & 0xff);
//   // TODO: longer responses are possible.
//   uint8_t len = (pd_response >> 8) & 0xff;
//   ESP_LOGD(TAG, "PD response code: 0x%02X, len: 0x%02X", code, len);

//   switch (code) {
//     case ResponseCode::NO_RESPONSE:
//       ESP_LOGE(TAG, "No response");
//       return true;
//     case ResponseCode::SUCCESS:
//       ESP_LOGD(TAG, "Success");
//       return true;
//     case ResponseCode::INVALID_CMD:
//       ESP_LOGD(TAG, "Invalid command");
//       return true;
//     case ResponseCode::NOT_SUPPORTED:
//       ESP_LOGE(TAG, "Not supported");
//       return true;
//     case ResponseCode::TRANSACTION_FAILED:
//       ESP_LOGE(TAG, "Transaction failed");
//       return true;
//     case ResponseCode::PD_CMD_FAILED:
//       ESP_LOGE(TAG, "PD command failed");
//       return true;
//     case ResponseCode::PS_READY:
//       ESP_LOGD(TAG, "PS ready");
//       return true;
//     case ResponseCode::PD_NEGOTIATION_COMPLETE:
//       return handle_pd_negotiation_complete(len);
//     case ResponseCode::ACCEPT_MSG_RECEIVED:
//       ESP_LOGD(TAG, "Accept message received");
//       return true;
//     case ResponseCode::REJECT_MSG_RECEIVED:
//       ESP_LOGE(TAG, "Reject message received");
//       return true;
//     case ResponseCode::SOURCE_CAPABILITIES:
//       return handle_source_capabilities(len);
//     case ResponseCode::TYPE_C_ERROR_RECOVERY:
//       ESP_LOGE(TAG, "Type C error recovery");
//       return true;
//     default:
//       ESP_LOGD(TAG, "Unhandled response code to %s: 0x%02X (full: 0x%08X)", pd_response & (0x1 << 7) ? "ASYNC" :
//       "CMD",
//                code, pd_response);
//       return false;
//   }
// }

// bool FUSB302::handle_source_capabilities(uint8_t len) {
//   ESP_LOGI(TAG, "Source capabilities received. Current state: %d", static_cast<int>(state_));

//   uint8_t n_pdos = (len - 4) / 4;
//   ESP_LOGD(TAG, "Number of PDOS: %d", n_pdos);

//   if (n_pdos > FUSB302_MAX_PDOS) {
//     ESP_LOGE(TAG, "Too many PDOS");
//     return false;
//   }

//   ESP_LOGD(TAG, "Reading PD response data from memory");

//   uint8_t buff[4 * FUSB302_MAX_PDOS + 4];
//   memset(buff, 0, sizeof(buff));
//   for (uint8_t i = 0; i < len; i++) {
//     if (this->read_register16(SWAP16(REG_READ_MEM_LO + i), &buff[i], 1)) {
//       ESP_LOGE(TAG, "Failed to read PD response data");
//       return false;
//     }
//   }

//   // Parse and store PDOs.
//   for (uint8_t i = 0; i < n_pdos; i++) {
//     uint32_t *pdo_data = (uint32_t *) &buff[i * 4 + 4];
//     pdos_[i] = parse_pdo(*pdo_data);
//     log_pdo(pdos_[i]);
//   }

//   ESP_LOGD(TAG, "Writing PD response data to memory. Current state: %d (req_caps: %d)", static_cast<int>(state_),
//            static_cast<int>(State::REQUESTED_CAPS));

//   // As per datasheet, to get ready for a power negotiation, we need to write the PD response data to memory.
//   for (uint8_t i = 0; i < len; i++) {
//     if (this->write_register16(SWAP16(REG_WRITE_MEM_LO + i), &buff[i], 1)) {
//       ESP_LOGE(TAG, "Failed to write PD response data");
//       return false;
//     }
//   }

//   // As per datasheet, we write the required header "SNKP" to memory.
//   const uint8_t header[] = {0x50, 0x4B, 0x4E, 0x53};
//   for (uint8_t i = 0; i < sizeof(header); i++) {
//     if (this->write_register16(SWAP16(REG_WRITE_MEM_LO + i), header + i, 1)) {
//       ESP_LOGE(TAG, "Failed to write PD response data");
//       return false;
//     }
//   }

//   // Can we find a suitable PDO?
//   selected_pdo_idx_ = -1;
//   for (int idx = 0; idx < n_pdos; idx++) {
//     const PDO *pdo = &pdos_[idx];
//     if (pdo->type == PDO::Type::FIXED && is_pdo_compatible(*pdo, this->power_requirement_)) {
//       selected_pdo_idx_ = idx;
//       break;
//     }
//   }

//   if (selected_pdo_idx_ == -1) {
//     // TODO: fatal.
//     ESP_LOGE(TAG, "No suitable fixed PDO found");
//     state_ = State::FAILURE;
//     return false;
//   }

//   ESP_LOGI(TAG, "Now we're cooking! Found suitable PDO:");
//   log_pdo(pdos_[selected_pdo_idx_]);

//   return true;
// }

// // We can probably just use the simpler SELECT_SINK_PDO register for this, but while I implemented this lower level
// // REQUEST to learn how it works while I unsuccessfully tried to get a PPS request to work. Well, it did work, and I
// // verified with a logic analyzer that the request is sent correctly and the source responds with both an ACCEPT and
// // READY message. But FUSB302 freaks out and issues a hard request upon the ACCEPT or RDY response for a PPS RDO :(.
// bool FUSB302::request_selected_fixed_pdo() {
//   if (selected_pdo_idx_ == -1) {
//     ESP_LOGE(TAG, "No suitable selected PDO -- aborting.");
//     state_ = State::FAILURE;
//     return false;
//   }

//   ESP_LOGD(TAG, "Requesting fixed PDO with index %d", selected_pdo_idx_);

//   uint32_t request = 0;

//   // Object position -- index + 1.
//   request |= (((selected_pdo_idx_ + 1) & 0x07) << 28);

//   // USB communications capability.
//   request |= (0x1 << 25);

//   // No USB suspend.
//   // request |= (0x1 << 24);

//   // Unchunked message supported.
//   request |= (0x1 << 23);

//   uint32_t current_10ma = 150;
//   request |= (current_10ma << 10);
//   request |= current_10ma;

//   ESP_LOGV(TAG, "Will send RDO: 0x%08X", request);
//   dump_rdo(&request, pdos_);

//   // Invert.
//   request = byteswap(request);
//   if (this->write_register16(REG_REQUEST, (uint8_t *) &request, sizeof(request))) {
//     // TODO: fatal.
//     ESP_LOGE(TAG, "Failed to write PD request");
//     return false;
//   }

//   state_ = State::REQUESTED_PDO;
//   return true;
// }

// bool FUSB302::handle_pd_negotiation_complete(uint8_t len) {
//   ESP_LOGI(TAG, "PD negotiation complete");
//   uint8_t buff[8];
//   if (len > sizeof(buff)) {
//     ESP_LOGE(TAG, "PD negotiation complete -- response is too long");
//     return false;
//   }

//   // Read from memory registers.
//   for (uint8_t i = 0; i < len; i++) {
//     if (this->read_register16(SWAP16(REG_READ_MEM_LO + i), &buff[i], 1)) {
//       ESP_LOGE(TAG, "Failed to read PD negotiation complete response");
//       return false;
//     }
//   }

//   if ((buff[0] & 0x1) == 0) {
//     ESP_LOGW(TAG, "Reason for pd negotiation failure: 0x%02X", buff[0] >> 2 & 0x3);
//   }

//   uint32_t *rdo_data = (uint32_t *) &buff[4];
//   ESP_LOGV(TAG, "Used request (RDO):");
//   dump_rdo(rdo_data, pdos_);

//   const PDO curr_pdo = get_current_pdo();
//   ESP_LOGV(TAG, "Currently active power delivery object (PDO):");
//   log_pdo(curr_pdo);

//   if (curr_pdo.type != PDO::Type::FIXED) {
//     ESP_LOGE(TAG, "Currently active PDO is not fixed -- something is very fishy. Aborting.");
//     state_ = State::FAILURE;
//     return false;
//   }

//   if (state_ == State::REQUESTED_CAPS) {
//     return request_selected_fixed_pdo();
//   } else if (state_ == State::REQUESTED_PDO) {
//     int available_power = curr_pdo.fixed.max_current_ma * curr_pdo.fixed.voltage_mv / (1000 * 1000);
//     ESP_LOGI(TAG, "Done. We got the power we wanted! Responsibly enjoy them %d Watts!!", available_power);
//     state_ = State::READY;
//   }

//   return true;
// }

}  // namespace fusb302
}  // namespace esphome
