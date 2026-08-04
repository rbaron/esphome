#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/display/display_buffer.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace gu128x64d7000 {

// Brightness levels accepted by the brightness control command. The display
// powers up at the brightest level.
static const uint8_t MIN_BRIGHTNESS = 1;
static const uint8_t MAX_BRIGHTNESS = 8;
static const uint8_t DEFAULT_BRIGHTNESS = MAX_BRIGHTNESS;

class GU128X64D7000 : public display::DisplayBuffer, public uart::UARTDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  void set_busy_pin(GPIOPin *busy_pin) { this->busy_pin_ = busy_pin; }

  // Turns the VFD power supply on/off. While off, no bitmap data is sent and
  // the panel is dark; turning it back on restores the current frame buffer.
  void set_power(bool power);
  bool is_powered() const { return this->power_; }

  // Brightness of the whole screen, from MIN_BRIGHTNESS (darkest) to
  // MAX_BRIGHTNESS (brightest).
  void set_brightness(uint8_t brightness);
  uint8_t get_brightness() const { return this->brightness_; }

  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_BINARY; }

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  int get_height_internal() override { return 64; }
  int get_width_internal() override { return 128; }

  void write_display_data_();

  GPIOPin *reset_pin_{nullptr};
  GPIOPin *busy_pin_{nullptr};
  bool power_{true};
  uint8_t brightness_{DEFAULT_BRIGHTNESS};
};

template<typename... Ts> class TurnOnAction : public Action<Ts...>, public Parented<GU128X64D7000> {
 public:
  void play(const Ts &...x) override { this->parent_->set_power(true); }
};

template<typename... Ts> class TurnOffAction : public Action<Ts...>, public Parented<GU128X64D7000> {
 public:
  void play(const Ts &...x) override { this->parent_->set_power(false); }
};

template<typename... Ts> class SetBrightnessAction : public Action<Ts...>, public Parented<GU128X64D7000> {
 public:
  TEMPLATABLE_VALUE(uint8_t, brightness)

  void play(const Ts &...x) override { this->parent_->set_brightness(this->brightness_.value(x...)); }
};

template<typename... Ts> class IsPoweredCondition : public Condition<Ts...>, public Parented<GU128X64D7000> {
 public:
  bool check(const Ts &...x) override { return this->parent_->is_powered(); }
};

}  // namespace gu128x64d7000
}  // namespace esphome
