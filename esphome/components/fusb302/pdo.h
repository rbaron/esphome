#pragma once

namespace esphome {
namespace fusb302 {

#define FUSB302_MAX_PDOS 7

// For PDO representation, see table 6.7 in the USB PD specs.
struct PDO {
  enum class Type {
    FIXED = 0b00,
    BATTERY = 0b01,
    VARIABLE = 0b10,
    AUGMENTED = 0b11,
  };

  struct Fixed {
    uint16_t voltage_mv;
    uint16_t max_current_ma;
  };

  struct Variable {
    uint16_t max_voltage_mv;
    uint16_t min_voltage_mv;
    uint16_t max_power_mw;
  };

  struct Augmented {
    enum class Type {
      SPR_PPS = 0b00,  // Standard Power programmable power supply.
      EPR_AVS = 0b01,  // Extended power range adjustable voltage supply.
      SPR_AVS = 0b10,  // Standard power range adjustable voltage supply.
    };
    Type type;

    // Table 6.13.
    struct SPR_PPS {
      uint16_t max_voltage_mv;
      uint16_t min_voltage_mv;
      uint16_t max_current_ma;
    };

    union {
      SPR_PPS spr_pps;
    };
  };

  Type type;
  bool parsed = false;
  union {
    Fixed fixed;
    Variable variable;
    Augmented augmented;
  };
};

struct PowerRequirement {
  uint16_t voltage_mv;
  uint16_t current_ma;
};

PDO parse_pdo(uint32_t data);

bool is_pdo_compatible(const PDO &pdo, const PowerRequirement &power_requirement);

void log_pdo(const PDO &pdo);

}  // namespace fusb302
}  // namespace esphome
