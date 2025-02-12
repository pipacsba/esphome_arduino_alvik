
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
          uint8_t battery_regs[] = {0, 0};
          if ((this->write(&BATTERY_REGISTER, 1, false) != i2c::ERROR_OK) || !this->read_bytes_raw(battery_regs, 2)) {
                ESP_LOGE(TAG, "Unable to i2c battery data");
                this->mark_failed();
                return;
          }
          uint16_t battery_val = = encode_uint16(battery_regs[1], battery_regs[0]);
          float battery_soc = battery_val * 0.00390625;
          ESP_LOGD(TAG, "Read battery data: %d, %0.1f", battery_val, battery_soc);
    }

    void AlvikComponent::loop() {
        uint32_t now = millis();
    }

    void AlvikComponent::dump_config() {
      ESP_LOGCONFIG(TAG, "AlvikComponent  : something is done!");
    }

    void AlvikEnableSwitch::write_state(bool state)
    {
    }

}  // namespace alvik
}  // namespace esphome
