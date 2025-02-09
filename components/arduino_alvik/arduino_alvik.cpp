#include "Arduino_Alvik.h"
#include "arduino_alvik.h"

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
}
  

}  // namespace alvik
}  // namespace esphome
