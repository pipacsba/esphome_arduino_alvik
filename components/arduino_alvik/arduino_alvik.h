#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/components/number/number.h"
//#include "HunterRoam.h"
#include <vector>

namespace esphome {
namespace hunterwifi {

#define START_INTERVAL 900
#define SHORT_INTERVAL 208
#define LONG_INTERVAL 1875
#define HIGH true
#define LOW false

class HunterWifiComponent;      // this component
class HunterZoneSwitch;         // switches representing any valve / zones
class HunterRoam;               // bus handling

class HunterRoam : public Component {
  public:
    HunterRoam(InternalGPIOPin *pin);
    uint8_t stopZone(uint8_t zone);
    uint8_t startZone(uint8_t zone, uint8_t time);
    uint8_t startProgram(uint8_t num);
    std::string errorHint(uint8_t error);

  private:
    InternalGPIOPin *pin_;
    void hunterBitfield(std::vector <uint8_t> &bits, uint8_t pos, uint8_t val, uint8_t len);
    void writeBus(std::vector<uint8_t> buffer, bool extrabit);
    void sendLow(void);
    void sendHigh(void);
};

// define struct for zone valves
struct HunterValve {
  HunterZoneSwitch *valve_switch;
  uint16_t zone_number;
  uint16_t max_duration;
  number::Number *duration_number_name{nullptr};
}; 
 
//main hunterwifi component (controller)
//this also maybe not even needed, not used for anything
class HunterWifiComponent  : public Component {
 public:
  // add pin
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void setup() override;
  void dump_config() override;
  void add_valve(HunterZoneSwitch *valve_sw, uint16_t zone_number, uint16_t max_duration, number::Number *duration_number_name); 
  void add_valve(HunterZoneSwitch *valve_sw, uint16_t zone_number, uint16_t max_duration); 
 
  /// returns a pointer to a valve's switch object
  HunterZoneSwitch *valve_switch(size_t valve_number);
  /// returns the number of valves the controller is configured with
  size_t number_of_valves();
  
 protected:
 
  /// Sprinkler valve objects
  std::vector<HunterValve> valve_;
 
  friend HunterZoneSwitch;

  InternalGPIOPin *pin_;
  HunterRoam *hunter_roam_;

};

//create switch for zones
class HunterZoneSwitch : public switch_::Switch, public Component {
 public:
  //HunterZoneSwitch();

  void setup() override;
  void dump_config() override;
  //define pin number to use for REM communication
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  //define the zone number as used in Huner device pinout
  void set_zone(uint8_t zone) { zone_ = zone; }
  //set maximum zone sprinkler duration
  void set_max_duration(uint8_t max_duration) { max_duration_ = max_duration; }
  //set duration number id
  void set_duration_number_name(number::Number *duration_number_name) {duration_number_name_ = duration_number_name;}
  
  //flexibility is everythin
  void set_state_lambda(std::function<optional<bool>()> &&f);
  void loop() override;

  float get_setup_priority() const override;

 protected:
  /** Write the given state to hardware. You should implement this
   * abstract method if you want to create your own switch.
   *
   * In the implementation of this method, you should also call
   * publish_state to acknowledge that the state was written to the hardware.
   *
   * @param state The state to write. Inversion is already applied if user specified it.
   */
  void write_state(bool state) override;

  InternalGPIOPin *pin_;
  HunterRoam *hunter_roam_;
  uint8_t zone_;
  uint8_t max_duration_;
  number::Number *duration_number_name_{nullptr};

  optional<std::function<optional<bool>()>> f_;
};

 
}  // namespace hunterwifi
}  // namespace esphome
