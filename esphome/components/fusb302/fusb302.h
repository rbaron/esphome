#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/fusb302/pdo.h"

namespace esphome {
namespace fusb302 {

enum class State {
  INITIALIZING = 0x0,
  REQUESTED_CAPS,
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

  // void set_vbus_voltage_sensor(sensor::Sensor *vbus_voltage_sensor) {
  //   this->vbus_voltage_sensor_ = vbus_voltage_sensor;
  // }

 private:
  State state_{State::INITIALIZING};

  // From config.
  PowerRequirement power_requirement_;

  // PDOs received from the source.
  // PDO pdos_[FUSB302_MAX_PDOS];

  // The selected PDO that satisfies the power requirement. Index into pdos_, as we need to actually send an index as
  // part of the RDO.
  // int selected_pdo_idx_{-1};

  // Interrupt pin.
  InternalGPIOPin *int_pin_{nullptr};
  bool interrupt_pending_{false};

  // Methods.
  bool process_interrupt();
  bool read_fifo();
  bool handle_msg(uint8_t msg_type, uint8_t n_objs, uint32_t *objs);
  // PDO get_current_pdo();
  // bool handle_event_status(uint32_t event_status);
  // bool handle_pd_response(uint32_t pd_response);
  // bool handle_source_capabilities(uint8_t len);
  // bool handle_pd_negotiation_complete(uint8_t len);
  // bool request_selected_fixed_pdo();

  // Handles interrupts.
  static void ISR(FUSB302 *instance);

  // sensor::Sensor *vbus_voltage_sensor_{nullptr};
};

}  // namespace fusb302
}  // namespace esphome
