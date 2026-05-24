#pragma once

#ifdef USE_ESP32

#include "esphome/components/display/display.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/time.h"

#include <esp_timer.h>

#include <array>

namespace esphome::ld8035_clock {

class LD8035Clock;
using ld8035_clock_writer_t = display::DisplayWriter<LD8035Clock>;

class LD8035Clock : public PollingComponent {
 public:
  static constexpr uint8_t NUM_DIGITS = 4;

  void setup() override;
  void dump_config() override;
  void update() override;
  void on_shutdown() override;
  float get_setup_priority() const override;

  void set_ser_pin(GPIOPin *pin) { this->ser_pin_ = pin; }
  void set_srclk_pin(GPIOPin *pin) { this->srclk_pin_ = pin; }
  void set_rclk_pin(GPIOPin *pin) { this->rclk_pin_ = pin; }
  void set_oe_pin(GPIOPin *pin) { this->oe_pin_ = pin; }
  void set_dp_pin(GPIOPin *pin) { this->dp_pin_ = pin; }
  void set_grid_pin(uint8_t index, GPIOPin *pin) { this->grid_pins_[index] = pin; }
  void set_scan_interval_us(uint32_t us) { this->scan_interval_us_ = us; }
  void set_writer(ld8035_clock_writer_t &&writer) { this->writer_ = std::move(writer); }

  /// Print `str` starting at digit `start_pos`. A '.' sets the DP on the
  /// previous digit instead of consuming a position.
  uint8_t print(uint8_t start_pos, const char *str);
  uint8_t print(const char *str) { return this->print(0, str); }
  uint8_t printf(uint8_t pos, const char *format, ...) __attribute__((format(printf, 3, 4)));
  uint8_t printf(const char *format, ...) __attribute__((format(printf, 2, 3)));
  uint8_t strftime(uint8_t pos, const char *format, ESPTime time) __attribute__((format(strftime, 3, 0)));
  uint8_t strftime(const char *format, ESPTime time) { return this->strftime(0, format, time); }

 protected:
  static void scan_tick_trampoline_(void *arg);
  void scan_tick_();
  void shift_byte_(uint8_t b);
  static uint8_t remap_to_shift_register_(uint8_t seg);

  GPIOPin *ser_pin_{nullptr};
  GPIOPin *srclk_pin_{nullptr};
  GPIOPin *rclk_pin_{nullptr};
  GPIOPin *oe_pin_{nullptr};
  GPIOPin *dp_pin_{nullptr};
  std::array<GPIOPin *, NUM_DIGITS> grid_pins_{};

  uint32_t scan_interval_us_{2000};
  esp_timer_handle_t scan_timer_{nullptr};

  // Logical segment buffer filled by the writer lambda via print().
  // Bit layout: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, 7=DASH (LD8035 extra bar).
  std::array<uint8_t, NUM_DIGITS> buffer_{};
  uint8_t dp_mask_{0};
  uint8_t active_digit_{0};

  ld8035_clock_writer_t writer_{};
};

}  // namespace esphome::ld8035_clock

#endif  // USE_ESP32
