#pragma once

namespace esphome {
namespace fusb302 {

// Logs a parsed PDO (Power Data Object) using one of the ESP_LOG* macros.
#define FUSB302_LOG_PDO(LOG, pdo) \
  do { \
    if (pdo.type == PDO::Type::FIXED) { \
      LOG(TAG, "Fixed PDO: %d mV, %d mA, EPR mode capable: %s, Unchunked ext msg support: %s", pdo.fixed.voltage_mv, \
          pdo.fixed.max_current_ma, pdo.fixed.epr_mode_capable ? "yes" : "no", \
          pdo.fixed.unchunked_ext_msg_supported ? "yes" : "no"); \
    } else if (pdo.type == PDO::Type::VARIABLE) { \
      LOG(TAG, "Variable PDO: %d-%d mV, %d mW", pdo.variable.min_voltage_mv, pdo.variable.max_voltage_mv, \
          pdo.variable.max_power_mw); \
    } else if (pdo.type == PDO::Type::AUGMENTED) { \
      if (pdo.augmented.type == PDO::Augmented::Type::SPR_PPS) { \
        LOG(TAG, "Augmented SPR_PPS PDO: %d - %d mV, %d mA", pdo.augmented.spr_pps.min_voltage_mv, \
            pdo.augmented.spr_pps.max_voltage_mv, pdo.augmented.spr_pps.max_current_ma); \
      } else { \
        LOG(TAG, "Unsupported augmented PDO type: %d", static_cast<int>(pdo.augmented.type)); \
      } \
    } else { \
      LOG(TAG, "Unsupported PDO type"); \
    } \
  } while (0)

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
    bool unchunked_ext_msg_supported;
    bool epr_mode_capable;
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

}  // namespace fusb302
}  // namespace esphome
