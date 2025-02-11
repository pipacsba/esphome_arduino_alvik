
#pragma once

#ifndef __ALVIK_ESPHOME__
#define __ALVIK_ESPHOME__

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/components/number/number.h"
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
  void set_fw_sensor(text_sensor::TextSensor *sensor1) { fw_version_sensor_ = sensor1; }
  void set_lib_sensor(text_sensor::TextSensor *sensor1) { lib_version_sensor_ = sensor1; }

 protected:
  Arduino_Alvik alvik;
  uint8_t battery_;
  sensor::Sensor *battery_sensor_;
  text_sensor::TextSensor *fw_version_sensor_;
  text_sensor::TextSensor *lib_version_sensor_;
  
  
 
};

 
}  // namespace alvik
}  // namespace esphome

#endif
