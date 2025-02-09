#include "Arduino_Alvik.h"

#include "esphome/core/entity_base.h"
#include "esphome/core/application.h"
#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <utility>

namespace esphome {
namespace alvik {

static const char *const TAG = "arduinoalvik";

//flexibility is everything
void AlvikComponent::loop() {
  if (!this->f_.has_value())
    return;
  auto s = (*this->f_)();
  if (!s.has_value())
    return;

  this->publish_state(*s);
}

  
void AlvikComponent::setup() {
  alvik.begin();
}

void AlvikComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HunterWifiComponent  :");
  LOG_PIN("  Pin: ", this->pin_);
  for (size_t valve_number = 0; valve_number < this->number_of_valves(); valve_number++) {
    ESP_LOGCONFIG(TAG, "  Valve %u:", valve_number);
    ESP_LOGCONFIG(TAG, "    Name: %s", this->valve_[valve_number].valve_switch->get_name().c_str());
    ESP_LOGCONFIG(TAG, "    Zone: %u", this->valve_[valve_number].zone_number);
    ESP_LOGCONFIG(TAG, "    Max_Duration: %u", this->valve_[valve_number].max_duration);
    if (this->valve_[valve_number].duration_number_name)
    {
      ESP_LOGCONFIG(TAG, "    Duration Number Name: %s", this->valve_[valve_number].duration_number_name->get_name().c_str());
    }
  }
}
  

}  // namespace alvik
}  // namespace esphome
