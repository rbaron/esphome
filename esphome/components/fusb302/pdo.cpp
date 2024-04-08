#include "esphome/core/log.h"
#include "pdo.h"

#include <optional>

static const char *TAG = "fusb302.pdo";

namespace esphome {
namespace fusb302 {

PDO parse_pdo(uint32_t data) {
  PDO pdo;
  pdo.type = static_cast<PDO::Type>(data >> 30);
  if (pdo.type == PDO::Type::FIXED) {
    pdo.fixed.voltage_mv = ((data >> 10) & 0x3FF) * 50;
    pdo.fixed.max_current_ma = (data & 0x3FF) * 10;
    pdo.parsed = true;
    return pdo;
  } else if (pdo.type == PDO::Type::VARIABLE) {
    pdo.variable.max_voltage_mv = ((data >> 20) & 0x3FF) * 50;
    pdo.variable.min_voltage_mv = ((data >> 10) & 0x3FF) * 50;
    pdo.variable.max_power_mw = (data & 0x3FF) * 250;
    pdo.parsed = true;
    return pdo;
  } else if (pdo.type == PDO::Type::AUGMENTED) {
    pdo.augmented.type = static_cast<PDO::Augmented::Type>((data >> 28) & 0x3);
    if (pdo.augmented.type == PDO::Augmented::Type::SPR_PPS) {
      pdo.augmented.spr_pps.max_voltage_mv = ((data >> 17) & 0xFF) * 100;
      pdo.augmented.spr_pps.min_voltage_mv = ((data >> 8) & 0xFF) * 100;
      pdo.augmented.spr_pps.max_current_ma = (data & 0x7F) * 50;
      pdo.parsed = true;
      return pdo;
    } else {
      ESP_LOGW(TAG, "Unsupported augmented PDO type: %d", static_cast<int>(pdo.augmented.type));
      return pdo;
    }
  }
  ESP_LOGW(TAG, "Unsupported PDO type: %d", static_cast<int>(pdo.type));
  return pdo;
}

bool is_pdo_compatible(const PDO &pdo, const PowerRequirement &power_requirement) {
  if (pdo.type == PDO::Type::FIXED) {
    return pdo.fixed.voltage_mv == power_requirement.voltage_mv &&
           pdo.fixed.max_current_ma >= power_requirement.current_ma;
  } else if (pdo.type == PDO::Type::AUGMENTED) {
    if (pdo.augmented.type == PDO::Augmented::Type::SPR_PPS) {
      return pdo.augmented.spr_pps.max_voltage_mv >= power_requirement.voltage_mv &&
             pdo.augmented.spr_pps.min_voltage_mv <= power_requirement.voltage_mv &&
             pdo.augmented.spr_pps.max_current_ma >= power_requirement.current_ma;
    }
  }
  ESP_LOGW(TAG, "Unsupported PDO type: %d, assuming not compatible", static_cast<int>(pdo.type));
  return false;
}

void log_pdo(const PDO &pdo) {
  if (pdo.type == PDO::Type::FIXED) {
    ESP_LOGI(TAG, "Fixed PDO: %d mV, %d mA", pdo.fixed.voltage_mv, pdo.fixed.max_current_ma);
  } else if (pdo.type == PDO::Type::VARIABLE) {
    ESP_LOGI(TAG, "Variable PDO: %d-%d mV, %d mW", pdo.variable.min_voltage_mv, pdo.variable.max_voltage_mv,
             pdo.variable.max_power_mw);
  } else if (pdo.type == PDO::Type::AUGMENTED) {
    if (pdo.augmented.type == PDO::Augmented::Type::SPR_PPS) {
      ESP_LOGI(TAG, "Augmented SPR_PPS PDO: %d - %d mV, %d mA", pdo.augmented.spr_pps.min_voltage_mv,
               pdo.augmented.spr_pps.max_voltage_mv, pdo.augmented.spr_pps.max_current_ma);
    } else {
      ESP_LOGW(TAG, "Unsupported augmented PDO type: %d", static_cast<int>(pdo.augmented.type));
    }
  } else {
    ESP_LOGW(TAG, "Unsupported PDO type");
  }
}

}  // namespace fusb302
}  // namespace esphome
