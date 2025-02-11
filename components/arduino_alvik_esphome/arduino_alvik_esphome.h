
#pragma once

#ifndef __ALVIK_ESPHOME__
#define __ALVIK_ESPHOME__

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "Arduino_Alvik.h"
#include <vector>

namespace esphome {
namespace alvik {

class AlvikComponent;      // this component
 
//main hunterwifi component (controller)
//this also maybe not even needed, not used for anything
class AlvikComponent  : public Component {
  public:

   void setup() override;
   void dump_config() override;
 
   void loop() override;
 
   void set_battery_sensor(sensor::Sensor *sensor1) { battery_sensor_ = sensor1; }
   void set_alive_sensor(sensor::Sensor *sensor1) { alvik_alive_ = sensor1; }
   void set_fw_sensor(text_sensor::TextSensor *sensor1) { fw_version_sensor_ = sensor1; }
   void set_lib_sensor(text_sensor::TextSensor *sensor1) { lib_version_sensor_ = sensor1; }
   void set_enable_alvik_switch(switch_::Switch *sw) { enable_alvik_switch_ = sw; }

  protected:
   Arduino_Alvik alvik;
   uint8_t battery_;
   sensor::Sensor *battery_sensor_;
   sensor::Sensor *alvik_alive_;
   text_sensor::TextSensor *fw_version_sensor_;
   text_sensor::TextSensor *lib_version_sensor_;

   switch_::Switch *enable_alvik_switch_;

};

class PowerFeatherSwitch : public switch_::Switch, public Parented<PowerFeatherMainboard>, public PowerFeatherUpdateable
{
  public:
    AlvikSwitch() = default;

  protected:
    void write_state(bool state) override
    {
       this->parent_->send_task_update(update);
       this->publish_state(state);
    }
};

 
}  // namespace alvik
}  // namespace esphome

#endif
