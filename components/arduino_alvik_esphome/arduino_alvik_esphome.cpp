
//#include "Arduino_Alvik.h"
#include "arduino_alvik_esphome.h"

#include "esphome/core/entity_base.h"
#include "esphome/core/application.h"
#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <utility>

namespace esphome {
namespace alvik {

static const char *const TAG = "arduinoalvik";
 
void AlvikComponent::setup() {
  this->alvik.begin();
}

void AlvikComponent::loop() {
  this->alvik.is_on();
}

void AlvikComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "AlvikComponent  :");
}
  

}  // namespace alvik
}  // namespace esphome
