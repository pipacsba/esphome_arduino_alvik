#define D3 GPIO6
#define D4 GPIO7
#define LED_RED GPIO46
#define LED_GREEN GPIO0


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
 
void AlvikComponent::setup() {
  this->alvik.begin();
}

void AlvikComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HunterWifiComponent  :");
}
  

}  // namespace alvik
}  // namespace esphome
