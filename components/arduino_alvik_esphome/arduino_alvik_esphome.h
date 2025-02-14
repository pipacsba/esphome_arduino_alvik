
#pragma once

#ifndef __ALVIK_ESPHOME__
#define __ALVIK_ESPHOME__

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/uart/uart.h"
#include "ucPack.h"
#include "default_colors.h"
//include "Arduino_Alvik.h"
#include <vector>

namespace esphome {
namespace alvik {

#define NO_ACK 0xFF

const uint8_t BATTERY_REGISTER          = 0x06;


//class AlvikComponent  : public Component, public i2c::I2CDevice, public uart::UARTDevice {
class AlvikComponent  : public Component, public uart::UARTDevice {
  public:
    ucPack * packeter = new ucPack(200);
    bool read_message();
    int parse_message();
    uint8_t b;
    uint8_t code;
    uint8_t msg_size;

    float battery;
    float battery_soc;
    uint16_t battery_val = 0;
    uint8_t battery_v[2];
    bool battery_is_charging;

    uint8_t last_ack;
    uint8_t waiting_ack;

    void setup() override;
    void dump_config() override;
    void set_check_stm32_pin(GPIOPin *pin) { stm32_pin_ = pin; };
    void set_check_nano_pin(GPIOPin *pin) { nano_pin_ = pin; };
    void set_stm32_state(bool ison) { stm32_is_on_ = ison; }
    void set_cycle(int a_cycle) { cycle_ = a_cycle; };

    void loop() override;
  
    void set_battery_sensor(sensor::Sensor *sensor1) { battery_sensor_ = sensor1; }
    void set_alive_sensor(sensor::Sensor *sensor1) { alvik_alive_sensor_ = sensor1; }
    void set_fw_sensor(text_sensor::TextSensor *sensor1) { fw_version_sensor_ = sensor1; }
    void set_lib_sensor(text_sensor::TextSensor *sensor1) { lib_version_sensor_ = sensor1; }
    void set_enable_alvik_switch(switch_::Switch *sw) { enable_alvik_switch_ = sw; }
    
    float get_setup_priority() const override { return setup_priority::DATA; }

    void get_wheels_speed(float & left, float & right);
    void set_wheels_speed(const float left, const float right);

    void get_wheels_position(float & left, float & right);
    void set_wheels_position(const float left, const float right, const bool blocking = true);

    void get_drive_speed(float & linear, float & angular);
    void drive(const float linear, const float angular);

    void get_pose(float & x, float & y, float & theta);
    void reset_pose(const float x = 0.0, const float y = 0.0, const float theta = 0.0);

    bool is_target_reached();
    void rotate(const float angle, const bool blocking = true);
    void move(const float distance);   // distance [mm]

    void brake();
    

    void get_line_sensors(int & left, int & center, int & right);
    void get_orientation(float & roll, float & pitch, float & yaw);
    void get_accelerations(float & x, float & y, float & z);
    void get_gyros(float & x, float & y, float & z);
    void get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz);
    bool get_shake();
    // String get_tilt();

    void get_distance(float & left, float & center_left, float & center, float & center_right, float & right);
    float get_distance_top();
    float get_distance_bottom();


    bool get_touch_any();
    bool get_touch_ok();
    bool get_touch_cancel();
    bool get_touch_center();
    bool get_touch_up();
    bool get_touch_left();
    bool get_touch_down();
    bool get_touch_right();


    void set_builtin_led(const bool value);
    void set_illuminator(const bool value);

    void set_servo_positions(const uint8_t a_position, const uint8_t b_position);
    void get_servo_positions(int & a_position, int & b_position);

    void set_behaviour(const uint8_t behaviour);
    void set_stm32_fw_compatible(bool compatible) { stm32_fw_compatible_ = compatible; };

  protected:
    int cycle_;

    //0: init
    //1: first ack arrived
    //2: firmware compatible
    int alvik_state_;

    GPIOPin *stm32_pin_{nullptr};
    GPIOPin *nano_pin_{nullptr};
    bool stm32_is_on_;
    bool stm32_fw_compatible_;

    std::string alvik_command_list_;
    int last_command_time_;

    //ucPack * packeter;
    //uint8_t msg_size;

    //uint8_t last_ack;
    //uint8_t waiting_ack;

    std::array<uint8_t, 3> fw_version;
    std::array<uint8_t, 3> lib_version;

    uint8_t led_state;
    int16_t line_sensors[3];

    int16_t color_sensor[3];
    uint16_t white_cal[3];
    uint16_t black_cal[3];
    float rgb_normalized[3];
    float hsv[3];

    uint8_t servo_positions[2];

    float orientation[3];
    uint8_t move_bits;

    float imu[6];

    int16_t distances[7];

    uint8_t touch, touch_bits;

    float joints_velocity[2];

    float joints_position[2];

    float robot_velocity[2];

    float robot_pose[3];

    uint8_t battery_;

    sensor::Sensor *battery_sensor_;
    sensor::Sensor *alvik_alive_sensor_;

    text_sensor::TextSensor *fw_version_sensor_;
    text_sensor::TextSensor *lib_version_sensor_;
 
    switch_::Switch *enable_alvik_switch_;

    static void update_task_(void *param);

};

class AlvikEnableSwitch : public switch_::Switch, public Parented<AlvikComponent> {
 public:
  AlvikEnableSwitch() = default;

 protected:
  void write_state(bool state) override;
};
 
}  // namespace alvik
}  // namespace esphome

#endif


/*
     _            _       _             
    / \   _ __ __| |_   _(_)_ __   ___  
   / _ \ | '__/ _` | | | | | '_ \ / _ \ 
  / ___ \| | | (_| | |_| | | | | | (_) |
 /_/   \_\_|  \__,_|\__,_|_|_| |_|\___/ 
     _    _       _ _                   
    / \  | |_   _(_) | __               
   / _ \ | \ \ / / | |/ /               
  / ___ \| |\ V /| |   <                
 /_/   \_\_| \_/ |_|_|\_\   

+---+----------------------------+--+----------+
|   |      ()             ()     |  |          |
|   |            \__/            |  |   /---\  |
|    \__________________________/   |   |   |  |
|                                   |   |   |  |
+-----------------------------------+---|   |--+
   \\\___/                            \\\___/   
*/
