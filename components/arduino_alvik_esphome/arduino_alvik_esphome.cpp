
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
      if (this->alvik_alive_ != nullptr)
          this->alvik_alive_->publish_state(0);
    }

    void AlvikComponent::loop() {
      if (this->alvik_alive_->state == 0)
      {
          bool is_on = this->alvik.is_on();
          if (is_on)
             this->alvik_alive_->publish_state(1);
      }
    }

    void AlvikComponent::dump_config() {
      ESP_LOGCONFIG(TAG, "AlvikComponent  : something is done!");
    }

    void AlvikComponent::update_task_(void *param)
    {
      alvik::AlvikComponent *alvik_component =
        reinterpret_cast<alvik::AlvikComponent *>(param);

      while (true)
      {
        TaskUpdate update;
        update.type = TaskUpdateType::SENSORS;
        xQueueReceive(alvik_component->update_task_queue_, &update, portMAX_DELAY);
        switch (update.type)
        {
        case TaskUpdateType::ENABLE_EN:
          PowerFeather::Board.setEN(powerfeather_mainboard->enable_EN_);
          break;
        default:
        {
          ESP_LOGD(TAG, "Recieved some update request");
          //powerfeather_mainboard->update_sensors_();
        }
        break;
        }
      }
    }



}  // namespace alvik
}  // namespace esphome
