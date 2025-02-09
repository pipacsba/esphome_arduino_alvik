#define D3 GPIO6
#define D4 GPIO7
#define LED_RED GPIO46
#define LED_GREEN GPIO0

#pragma once

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

 protected:
  Arduino_Alvik alvik;
 
};

 
}  // namespace alvik
}  // namespace esphome
