#pragma once

#include <vector>
#include "esphome/core/optional.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/fusb302/pdo.h"
#include "esphome/components/fusb302/consts.h"

namespace esphome {
namespace fusb302 {

enum class State {
  STARTUP = 0x0,
  WAIT_FOR_CAPABILITIES,
  EVALUATE_CAPABILITY,
  SELECT_CAPABILITY,
  TRANSITION_SINK,
  READY,
  FAILURE,
};

struct FIFOMsg {
  enum class Destination { UNKNOWN, SOP };
  Destination destination = Destination::UNKNOWN;
  uint16_t header;
  uint8_t msg_type;
  uint8_t n_objs;
  bool extended;
  uint32_t objs[kMaxPDOS];
};

class FUSB302 : public i2c::I2CDevice, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_interrupt_pin(InternalGPIOPin *int_pin) { this->int_pin_ = int_pin; }

  void set_start_power_negotiation_on_boot(bool start) { this->start_power_negotiation_on_boot_ = start; }

  void set_voltage_requirement(uint16_t voltage_mv);

  void set_current_requirement(uint16_t current_ma);

  void add_on_pd_negotiation_success_callback(std::function<void(bool)> &&callback) {
    this->on_pd_negotiation_success_callback_.add(std::move(callback));
  }

  void add_on_pd_negotiation_failure_callback(std::function<void(bool)> &&callback) {
    this->on_pd_negotiation_failure_callback_.add(std::move(callback));
  }

  void start_power_negotiation();

 private:
  State state_{State::STARTUP};

  HighFrequencyLoopRequester high_freq_loop_req_;

  // From config.
  PowerRequirement power_requirement_{
      .voltage_mv = 5000,
      .current_ma = 1000,
  };

  FIFOMsg fifo_msg_{};

  // Will be learned from messages received from the source.
  uint8_t pd_spec_;

  // PDOs received from the source.
  std::vector<PDO> pdos_;
  uint8_t n_pdos_{0};
  optional<uint8_t> selected_pdo_idx_;

  // Interrupt pin.
  InternalGPIOPin *int_pin_{nullptr};
  volatile bool interrupt_pending_{true};

  // Chunked message handling.
  struct {
    uint8_t data[kMaxExtendedMsgLen];
    size_t total_len;
    size_t current_len;
  } chunked_buffer_;
  bool send_chunk_request(uint8_t chunk_number);

  // Private member functions.
  bool process_interrupt();
  bool has_fifo_msg();
  bool read_fifo();
  bool handle_msg();
  bool handle_extended_msg();
  bool send_msg(uint8_t msg_type, uint8_t len, uint8_t *data, bool extended = false);
  bool parse_pdos(uint8_t n_pdos, uint32_t *pdos);
  bool evaluate_capabilities();
  bool request_pdo();
  bool send_soft_reset();
  bool measure_cc_pin(uint8_t cc_pin, uint8_t *voltage_out);

  // Timer callbacks.
  void maybe_rerequest_pps_pdo();
  void maybe_send_epr_keepalive();

  // EPR mode.
  bool send_epr_mode_enter();
  bool send_epr_mode_exit();
  bool epr_mode_{false};

  Mutex mutex_{};

  // Wrapper around state change, so we can trigger callbacks.
  void enter_state(State state);

  // Handles interrupts.
  static void ISR(FUSB302 *instance);

  // Soft reset watchdog.
  static void Watchdog(FUSB302 *instance);

  bool start_power_negotiation_on_boot_ = true;
  bool power_negotiation_started_ = false;
  bool first_ready_state_ = true;

  CallbackManager<void(bool)> on_pd_negotiation_success_callback_{};
  CallbackManager<void(bool)> on_pd_negotiation_failure_callback_{};
};

}  // namespace fusb302
}  // namespace esphome
