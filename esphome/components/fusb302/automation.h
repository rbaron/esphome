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

}  // namespace fusb302
}  // namespace esphome