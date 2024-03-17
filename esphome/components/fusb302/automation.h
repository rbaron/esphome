#pragma once

#include "esphome/components/fusb302/fusb302.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fusb302 {

class OnPDNegotiationTrigger : public Trigger<bool> {
 public:
  OnPDNegotiationTrigger(FUSB302 *fusb302) {
    fusb302->add_on_pd_negotiation_callback([this](bool success) { this->trigger(success); });
  }
};

}  // namespace fusb302
}  // namespace esphome