#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/fusb302/pdo.h"

namespace esphome {
namespace fusb302 {

enum class State {
  INITIALIZING = 0x0,
  RECEIVED_CAPS,
  REQUESTED_PDO,
  READY,
  FAILURE,
};

class FUSB302 : public i2c::I2CDevice, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_interrupt_pin(InternalGPIOPin *int_pin) { this->int_pin_ = int_pin; }

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

 private:
  State state_{State::INITIALIZING};

  // From config.
  PowerRequirement power_requirement_;

  // PDOs received from the source.
  // PDO pdos_[FUSB302_MAX_PDOS];

  // Interrupt pin.
  InternalGPIOPin *int_pin_{nullptr};
  bool interrupt_pending_{false};

  // Methods.
  bool process_interrupt();
  bool read_fifo();
  bool send_msg(size_t len, uint8_t *data);
  bool handle_msg(uint8_t msg_type, uint8_t n_objs, uint32_t *objs);
  bool request_pdo(uint8_t pdo_idx);

  // Handles interrupts.
  static void ISR(FUSB302 *instance);

  // sensor::Sensor *vbus_voltage_sensor_{nullptr};

  CallbackManager<void(bool)> on_pd_negotiation_success_callback_{};
  CallbackManager<void(bool)> on_pd_negotiation_failure_callback_{};
};

}  // namespace fusb302
}  // namespace esphome
