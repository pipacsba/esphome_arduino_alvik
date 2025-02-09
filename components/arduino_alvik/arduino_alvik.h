#define D3 6
#define D4 7
#define LED_RED 46
#define LED_GREEN 45

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
