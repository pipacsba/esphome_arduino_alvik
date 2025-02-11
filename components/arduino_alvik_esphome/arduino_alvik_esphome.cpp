
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

    static uint32_t millis()
    {
      return esp_timer_get_time() / 1000;
    }

    void AlvikComponent::setup() {
      //this->alvik.begin();
      if (this->alvik_alive_sensor_ != nullptr)
          this->alvik_alive_sensor_->publish_state(0);
    }

    void AlvikComponent::loop() {
        uint32_t now = millis();
    }

    void AlvikComponent::dump_config() {
      ESP_LOGCONFIG(TAG, "AlvikComponent  : something is done!");
    }

    void write_state(bool state)
    {
        if (state)
        {
            this->parent->alvic.begin();
        }
    }

}  // namespace alvik
}  // namespace esphome
