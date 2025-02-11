
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
  if (this->alvik_alive_ != nullptr)
      this->alvik_alive_->publish_state(0);
}

void AlvikComponent::loop() {
  if (this->alvik_alive_.state == 0)
  {
      bool is_on = this->alvik.is_on();
      if (is_on)
         this->alvik_alive_->publish_state(1);
  }
}

void AlvikComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "AlvikComponent  :");
}
  

}  // namespace alvik
}  // namespace esphome
