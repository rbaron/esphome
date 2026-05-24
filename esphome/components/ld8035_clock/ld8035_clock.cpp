#ifdef USE_ESP32

#include "ld8035_clock.h"
#include "esphome/core/log.h"

#include <cstdarg>
#include <cstdio>

namespace esphome::ld8035_clock {

static const char *const TAG = "ld8035_clock";

static constexpr uint8_t SEG_A = 1 << 0;
static constexpr uint8_t SEG_B = 1 << 1;
static constexpr uint8_t SEG_C = 1 << 2;
static constexpr uint8_t SEG_D = 1 << 3;
static constexpr uint8_t SEG_E = 1 << 4;
static constexpr uint8_t SEG_F = 1 << 5;
static constexpr uint8_t SEG_G = 1 << 6;
static constexpr uint8_t SEG_DASH = 1 << 7;

static uint8_t segment_pattern_for_(char c) {
  if (c >= 'A' && c <= 'Z')
    c = static_cast<char>(c - 'A' + 'a');
  switch (c) {
    case '0':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case '1':
      return SEG_B | SEG_C;
    case '2':
      return SEG_A | SEG_B | SEG_D | SEG_E | SEG_G;
    case '3':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_G;
    case '4':
      return SEG_B | SEG_C | SEG_F | SEG_G;
    case '5':
      return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case '6':
      return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '7':
      return SEG_A | SEG_B | SEG_C;
    case '8':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '9':
      return SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;
    case 'a':
      return SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;
    case 'b':
      return SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case 'c':
      return SEG_A | SEG_D | SEG_E | SEG_F;
    case 'd':
      return SEG_B | SEG_C | SEG_D | SEG_E | SEG_G;
    case 'e':
      return SEG_A | SEG_D | SEG_E | SEG_F | SEG_G;
    case 'f':
      return SEG_A | SEG_E | SEG_F | SEG_G;
    case 'g':
      return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'h':
      return SEG_C | SEG_E | SEG_F | SEG_G;
    case 'i':
      return SEG_E | SEG_F;
    case 'j':
      return SEG_B | SEG_C | SEG_D | SEG_E;
    case 'l':
      return SEG_D | SEG_E | SEG_F;
    case 'n':
      return SEG_C | SEG_E | SEG_G;
    case 'o':
      return SEG_C | SEG_D | SEG_E | SEG_G;
    case 'p':
      return SEG_A | SEG_B | SEG_E | SEG_F | SEG_G;
    case 'r':
      return SEG_E | SEG_G;
    case 's':
      return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case 't':
      return SEG_D | SEG_E | SEG_F | SEG_G;
    case 'u':
      return SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'y':
      return SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;
    case '-':
      return SEG_G;
    case '_':
      return SEG_D;
    case ' ':
      return 0;
    default:
      return 0;
  }
}

float LD8035Clock::get_setup_priority() const { return setup_priority::PROCESSOR; }

void LD8035Clock::setup() {
  this->ser_pin_->setup();
  this->srclk_pin_->setup();
  this->rclk_pin_->setup();
  this->dp_pin_->setup();
  this->ser_pin_->digital_write(false);
  this->srclk_pin_->digital_write(false);
  this->rclk_pin_->digital_write(false);
  this->dp_pin_->digital_write(false);

  for (auto *grid : this->grid_pins_) {
    grid->setup();
    grid->digital_write(false);
  }

  if (this->oe_pin_ != nullptr) {
    this->oe_pin_->setup();
    this->oe_pin_->digital_write(true);  // /OE high = outputs Hi-Z (blanked)
  }

  // Clear the shift register so we don't start the first frame with garbage.
  this->shift_byte_(0);
  this->rclk_pin_->digital_write(true);
  this->rclk_pin_->digital_write(false);

  esp_timer_create_args_t args = {};
  args.callback = &LD8035Clock::scan_tick_trampoline_;
  args.arg = this;
  // TASK dispatch lets us call GPIOPin / FreeRTOS-aware code from the callback.
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "ld8035_scan";
  args.skip_unhandled_events = true;

  esp_err_t err = esp_timer_create(&args, &this->scan_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_create failed: %d", err);
    this->mark_failed();
    return;
  }
  err = esp_timer_start_periodic(this->scan_timer_, this->scan_interval_us_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
    this->mark_failed();
  }
}

void LD8035Clock::on_shutdown() {
  if (this->scan_timer_ != nullptr) {
    esp_timer_stop(this->scan_timer_);
    esp_timer_delete(this->scan_timer_);
    this->scan_timer_ = nullptr;
  }
  if (this->oe_pin_ != nullptr)
    this->oe_pin_->digital_write(true);
  for (auto *grid : this->grid_pins_)
    grid->digital_write(false);
  this->dp_pin_->digital_write(false);
}

void LD8035Clock::dump_config() {
  ESP_LOGCONFIG(TAG,
                "LD8035 Clock:\n"
                "  Digits: %u\n"
                "  Scan interval: %u us",
                NUM_DIGITS, this->scan_interval_us_);
  LOG_PIN("  SER (data):   ", this->ser_pin_);
  LOG_PIN("  SRCLK (clk):  ", this->srclk_pin_);
  LOG_PIN("  RCLK (latch): ", this->rclk_pin_);
  LOG_PIN("  /OE:          ", this->oe_pin_);
  LOG_PIN("  DP:           ", this->dp_pin_);
  for (size_t i = 0; i < this->grid_pins_.size(); i++) {
    ESP_LOGCONFIG(TAG, "  Grid %u:", static_cast<unsigned>(i));
    LOG_PIN("    ", this->grid_pins_[i]);
  }
  LOG_UPDATE_INTERVAL(this);
}

void LD8035Clock::update() {
  this->buffer_.fill(0);
  this->dp_mask_ = 0;
  if (this->writer_.has_value())
    (*this->writer_)(*this);
}

void LD8035Clock::scan_tick_trampoline_(void *arg) { static_cast<LD8035Clock *>(arg)->scan_tick_(); }

void LD8035Clock::scan_tick_() {
  // 1) BLANK the display so the about-to-be-shifted bits never show on the
  //    previously-active digit (no ghosting).
  if (this->oe_pin_ != nullptr) {
    this->oe_pin_->digital_write(true);
  } else {
    for (auto *grid : this->grid_pins_)
      grid->digital_write(false);
  }
  this->dp_pin_->digital_write(false);

  // 2) Advance to the next digit.
  this->active_digit_ = (this->active_digit_ + 1) % NUM_DIGITS;
  uint8_t seg = this->buffer_[this->active_digit_];

  // 3) Shift the new segment byte and latch it onto the 595's outputs.
  this->shift_byte_(remap_to_shift_register_(seg));
  this->rclk_pin_->digital_write(true);
  this->rclk_pin_->digital_write(false);

  // 4) DP is on a direct GPIO -- set it for this digit.
  this->dp_pin_->digital_write(((this->dp_mask_ >> this->active_digit_) & 1) != 0);

  // 5) UN-BLANK: enable the active grid (and only the active grid).
  if (this->oe_pin_ != nullptr) {
    for (uint8_t i = 0; i < NUM_DIGITS; i++)
      this->grid_pins_[i]->digital_write(i == this->active_digit_);
    this->oe_pin_->digital_write(false);
  } else {
    this->grid_pins_[this->active_digit_]->digital_write(true);
  }
}

void LD8035Clock::shift_byte_(uint8_t b) {
  for (int8_t i = 7; i >= 0; i--) {
    this->ser_pin_->digital_write(((b >> i) & 1) != 0);
    this->srclk_pin_->digital_write(true);
    this->srclk_pin_->digital_write(false);
  }
}

uint8_t LD8035Clock::remap_to_shift_register_(uint8_t seg) {
  // Physical 595 wiring (matches the existing prototype):
  //   Q0=C  Q1=D  Q2=E  Q3=F  Q4=G  Q5=A  Q6=B  Q7=DASH
  uint8_t out = 0;
  if (seg & SEG_C)
    out |= 1 << 0;
  if (seg & SEG_D)
    out |= 1 << 1;
  if (seg & SEG_E)
    out |= 1 << 2;
  if (seg & SEG_F)
    out |= 1 << 3;
  if (seg & SEG_G)
    out |= 1 << 4;
  if (seg & SEG_A)
    out |= 1 << 5;
  if (seg & SEG_B)
    out |= 1 << 6;
  if (seg & SEG_DASH)
    out |= 1 << 7;
  return out;
}

uint8_t LD8035Clock::print(uint8_t start_pos, const char *str) {
  // '.' and '~' are suffix-only: they OR the custom decimal point / custom
  // dash element onto the previously-emitted digit and do not consume a
  // position. Using them with no preceding digit is a usage error and is
  // logged + dropped. '-' is a regular character (segment G).
  uint8_t pos = start_pos;
  bool can_suffix = false;
  for (; *str != '\0'; str++) {
    if (*str == '.' || *str == '~') {
      if (!can_suffix) {
        ESP_LOGW(TAG, "'%c' is suffix-only and must follow a digit", *str);
        continue;
      }
      if (*str == '.') {
        this->dp_mask_ |= static_cast<uint8_t>(1 << (pos - 1));
      } else {
        this->buffer_[pos - 1] |= SEG_DASH;
      }
      continue;
    }
    if (pos >= NUM_DIGITS) {
      ESP_LOGW(TAG, "String too long for display");
      break;
    }
    this->buffer_[pos++] = segment_pattern_for_(*str);
    can_suffix = true;
  }
  return pos - start_pos;
}

uint8_t LD8035Clock::printf(uint8_t pos, const char *format, ...) {
  va_list arg;
  va_start(arg, format);
  char buffer[32];
  int ret = vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  if (ret > 0)
    return this->print(pos, buffer);
  return 0;
}

uint8_t LD8035Clock::printf(const char *format, ...) {
  va_list arg;
  va_start(arg, format);
  char buffer[32];
  int ret = vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  if (ret > 0)
    return this->print(buffer);
  return 0;
}

uint8_t LD8035Clock::strftime(uint8_t pos, const char *format, ESPTime time) {
  char buffer[32];
  size_t ret = time.strftime(buffer, sizeof(buffer), format);
  if (ret > 0)
    return this->print(pos, buffer);
  return 0;
}

}  // namespace esphome::ld8035_clock

#endif  // USE_ESP32
