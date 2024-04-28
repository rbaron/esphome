#pragma once

#include <vector>
#include "esphome/core/optional.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/fusb302/pdo.h"

namespace esphome {
namespace fusb302 {

enum class State {
  STARTUP = 0x0,
  WAIT_FOR_CAPABILITIES,
  EVALUATE_CAPABILITY,
  SELECT_CAPABILITY,
  TRANSITION_SINK,
  READY,

  // RECEIVED_CAPS,
  // REQUESTED_PDO,
  REQUESTED_SAFE_5V,
  // READY,
  FAILURE,
};

struct FIFOMsg {
  enum class Destination { UNKNOWN, SOP };
  Destination destination = Destination::UNKNOWN;
  uint16_t header;
  uint8_t msg_type;
  uint8_t n_objs;
  uint32_t objs[FUSB302_MAX_PDOS];
};

class FUSB302 : public i2c::I2CDevice, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_interrupt_pin(InternalGPIOPin *int_pin) { this->int_pin_ = int_pin; }

  void set_start_power_negotiation_on_boot(bool start) { this->start_power_negotiation_on_boot_ = start; }

  void set_power_requirement(uint16_t voltage_mv, uint16_t current_ma) {
    this->power_requirement_.voltage_mv = voltage_mv;
    this->power_requirement_.current_ma = current_ma;
  }

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
  PowerRequirement power_requirement_;

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

  // Methods.
  bool process_interrupt();
  bool has_fifo_msg();
  bool read_fifo();
  bool handle_msg();
  bool send_msg(uint8_t msg_type, uint8_t len, uint8_t *data);
  bool parse_pdos(uint8_t n_pdos, uint32_t *pdos);
  bool request_pdo();
  bool send_soft_reset();
  bool measure_cc_pin(uint8_t cc_pin, uint8_t *voltage_out);
  void maybe_rerequest_pps_pdo();

  // Reliability hacks.
  i2c::ErrorCode read_register_retry(uint8_t a_register, uint8_t *data, size_t len, bool stop = true);
  i2c::ErrorCode write_register_retry(uint8_t a_register, const uint8_t *data, size_t len, bool stop = true);
  bool read_byte_retry(uint8_t reg, uint8_t *value, bool stop = true);
  bool write_byte_retry(uint8_t reg, uint8_t value, bool stop = true);

  Mutex mutex_{};

  // Wrapper around state change, so we can trigger callbacks.
  void enter_state(State state);

  // Handles interrupts.
  static void ISR(FUSB302 *instance);

  // Soft reset watchdog.
  static void Watchdog(FUSB302 *instance);

  bool start_power_negotiation_on_boot_ = true;
  bool power_negotiation_started_ = false;

  CallbackManager<void(bool)> on_pd_negotiation_success_callback_{};
  CallbackManager<void(bool)> on_pd_negotiation_failure_callback_{};

  static void task(void *arg);
};

}  // namespace fusb302
}  // namespace esphome
