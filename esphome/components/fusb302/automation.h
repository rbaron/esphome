#pragma once

#include "esphome/components/fusb302/fusb302.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fusb302 {

class OnPDNegotiationSuccessTrigger : public Trigger<bool> {
 public:
  OnPDNegotiationSuccessTrigger(FUSB302 *fusb302) {
    fusb302->add_on_pd_negotiation_success_callback([this](bool success) { this->trigger(success); });
  }
};

class OnPDNegotiationFailureTrigger : public Trigger<bool> {
 public:
  OnPDNegotiationFailureTrigger(FUSB302 *fusb302) {
    fusb302->add_on_pd_negotiation_failure_callback([this](bool success) { this->trigger(success); });
  }
};

template<typename... Ts> class StartPowerNegotiationAction : public Action<Ts...> {
 public:
  explicit StartPowerNegotiationAction(FUSB302 *fusb302) : fusb302_(fusb302) {}
  void play(Ts... x) override { this->fusb302_->start_power_negotiation(); }

 protected:
  FUSB302 *fusb302_;
};

template<typename... Ts> class SetVoltageRequirementAction : public Action<Ts...> {
 public:
  explicit SetVoltageRequirementAction(FUSB302 *fusb302) : fusb302_(fusb302) {}

  TEMPLATABLE_VALUE(uint16_t, voltage)

  void play(Ts... x) override { this->fusb302_->set_voltage_requirement(this->voltage_.value(x...)); }

 protected:
  FUSB302 *fusb302_;
};

}  // namespace fusb302
}  // namespace esphome