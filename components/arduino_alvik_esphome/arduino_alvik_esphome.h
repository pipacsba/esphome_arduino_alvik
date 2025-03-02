
#pragma once

#ifndef __ALVIK_ESPHOME__
#define __ALVIK_ESPHOME__

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
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
//#define PI     3.14159265358979323846  /* pi */

// behaviours
#define BEHAVIOUR_DISABLED 0
#define BEHAVIOUR_ILLUMINATOR_RISE 1
#define BEHAVIOUR_BATTERY_ALERT 2

const uint8_t BATTERY_REGISTER       = 0x06;
const uint8_t CHARGE_THRESHOLD       = 97;

//COMPASS_SENSOR
const uint8_t CRA_REG_M              = 0x00;
const uint8_t CRB_REG_M              = 0x01;
const uint8_t M_REG_M                = 0x02;
const uint8_t M_REG_M_CONTINOUS      = 0x00;
const uint8_t M_REG_MEASUREMENT      = 0x03;
const uint8_t M_REG_MEASUREMENT_LEN  = 6;

const int16_t _lsm303Mag_Gauss_LSB_XY    = 1100;
const int16_t _lsm303Mag_Gauss_LSB_Z     =  980;
const float SENSORS_GAUSS_TO_MICROTESLA  =  100;


//ALVIK STATE MACHINE
const uint8_t ALVIK_STARTUP          =   0;
const uint8_t ALVIK_HW_RESET         =   1;
const uint8_t ALVIK_STM32_UP         =   2;
const uint8_t ALVIK_FIRST_ACK        =   3;
const uint8_t ALVIK_FW_COMPATIBLE    =   4;
const uint8_t ALVIK_EXTERNAL_SUPPLY  = 255;

//ALVIK TASK SCHEDULE
const uint8_t TASK_READ_UART       =   0;
const uint8_t TASK_PERFORM_ACTION  =   1;
const uint8_t TASK_WRITE_SENSOR    =   2;

//ALVIK ACTIONS
const uint8_t ACTION_PERFORM_COMMAND_LIST = 0;
const uint8_t ACTION_COLLECT_COMMAND_LIST = 1;
const uint8_t ACTION_FOLLOW = 2;
const uint8_t ACTION_NOT_SET = 255;


// LEFT and RIGHT LED COLORS
const uint8_t INTERNAL_LED = 1;
const uint8_t ILLUMINATOR  = 2;
const uint8_t LEFT_RED     = 4;
const uint8_t LEFT_GREEN   = 8;
const uint8_t LEFT_BLUE    = 16;
const uint8_t RIGHT_RED    = 32;
const uint8_t RIGHT_GREEN  = 64;
const uint8_t RIGHT_BLUE   = 128;

//class AlvikBatterySensor : public sensor::Sensor, public i2c::I2CDevice, public Parented<AlvikComponent>
class AlvikBatterySensor : public sensor::Sensor, public i2c::I2CDevice
{
  public:
    AlvikBatterySensor() = default;
};

class AlvikCompassSensor : public sensor::Sensor, public i2c::I2CDevice
{
  public:
    AlvikCompassSensor() = default;
};


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
    bool orientation_correction_enabled;

    uint8_t last_ack;
    uint8_t waiting_ack;

    void setup() override;
    void dump_config() override;
    void set_check_stm32_pin(GPIOPin *pin) { stm32_pin_ = pin; };
    void set_check_nano_pin(GPIOPin *pin) { nano_pin_ = pin; };
    void set_reset_stm32_pin(GPIOPin *pin) { reset_pin_ = pin; };
    void set_red_led_pin(GPIOPin *pin) { red_led_pin_ = pin; };
    void set_green_led_pin(GPIOPin *pin) { green_led_pin_ = pin; };
    void set_blue_led_pin(GPIOPin *pin) { blue_led_pin_ = pin; };
    void set_i2c_switch1_pin(GPIOPin *pin) { i2c_switch1_pin_ = pin; };
    void set_i2c_switch2_pin(GPIOPin *pin) { i2c_switch2_pin_ = pin; };
    void set_forward_move_distance(float a_distance) { forward_move_distance_ = a_distance; }
    void set_turn_degree(float an_angle) { turn_degree_ = an_angle; }

    void set_alvik_state(int a_state) { alvik_state_ = a_state; };

    void do_one_item_from_command_list(uint32_t now);
    void external_supply_measurement(bool ison);

    void change_alvik_left_right_leds(uint8_t change_led_state, bool onoff);


    void set_stm32_state(bool ison) { stm32_is_on_ = ison; }
    void set_cycle(int a_cycle) { cycle_ = a_cycle; };

    void loop() override;

    // SENSORS

    void set_compass_sensor(AlvikCompassSensor *sensor1) { compass_sensor_ = sensor1; }
    void set_compass_x_sensor(sensor::Sensor *sensor1) { compass_x_ = sensor1; }
    void set_compass_y_sensor(sensor::Sensor *sensor1) { compass_y_ = sensor1; }
    void set_compass_z_sensor(sensor::Sensor *sensor1) { compass_z_ = sensor1; }
    void set_battery_sensor(AlvikBatterySensor *sensor1) { battery_sensor_ = sensor1; }
    void set_alive_sensor(sensor::Sensor *sensor1) { alvik_alive_sensor_ = sensor1; }
    void set_action_sensor(sensor::Sensor *sensor1) { alvik_action_state_ = sensor1; }
    void set_pose_x_sensor(sensor::Sensor *sensor1) { pose_x_sensor_ = sensor1; }
    void set_pose_y_sensor(sensor::Sensor *sensor1) { pose_y_sensor_ = sensor1; }
    void set_pose_ang_sensor(sensor::Sensor *sensor1) { pose_ang_sensor_ = sensor1; }
    void set_roll_sensor(sensor::Sensor *sensor1) { roll_sensor_ = sensor1; }
    void set_pitch_sensor(sensor::Sensor *sensor1) { pitch_sensor_ = sensor1; }
    void set_yaw_sensor(sensor::Sensor *sensor1) { yaw_sensor_ = sensor1; }
    void set_yaw_est_sensor(sensor::Sensor *sensor1) { yaw_est_sensor_ = sensor1; }
    void set_joints_l_sensor(sensor::Sensor *sensor1) { joints_l_ = sensor1; }
    void set_joints_r_sensor(sensor::Sensor *sensor1) { joints_r_ = sensor1; }
    void set_distance_l_sensor(sensor::Sensor *sensor1) { distance_l_ = sensor1; }
    void set_distance_cl_sensor(sensor::Sensor *sensor1) { distance_cl_ = sensor1; }
    void set_distance_c_sensor(sensor::Sensor *sensor1) { distance_c_ = sensor1; }
    void set_distance_cr_sensor(sensor::Sensor *sensor1) { distance_cr_ = sensor1; }
    void set_distance_r_sensor(sensor::Sensor *sensor1) { distance_r_ = sensor1; }
    void set_distance_b_sensor(sensor::Sensor *sensor1) { distance_b_ = sensor1; }
    void set_distance_t_sensor(sensor::Sensor *sensor1) { distance_t_ = sensor1; }



    // NUMBERS
    void set_forward_distance_number(number::Number *a_number) { forward_distance_ = a_number; }
    void set_turn_degree_number(number::Number *a_number) { turn_degree_number_ = a_number; }

    //TEXT SENSORS
    void set_fw_sensor(text_sensor::TextSensor *sensor1) { fw_version_sensor_ = sensor1; }
    void set_lib_sensor(text_sensor::TextSensor *sensor1) { lib_version_sensor_ = sensor1; }
    void set_command_list_sensor(text_sensor::TextSensor *sensor1) { command_list_sensor_ = sensor1; }


    //SWITCHES
    void set_enable_alvik_switch(switch_::Switch *sw) { enable_alvik_switch_ = sw; }

    //BUTTONS
    void set_center_button(button::Button *bttn) { center_button_ = bttn; }
    void set_cancel_button(button::Button *bttn) { cancel_button_ = bttn; }
    void set_ok_button(button::Button *bttn) { ok_button_ = bttn; }
    void set_turnright_button(button::Button *bttn) { turn_right_button_ = bttn; }
    void set_turnleft_button(button::Button *bttn) { turn_left_button_ = bttn; }
    void set_backwards_button(button::Button *bttn) { backwards_button_ = bttn; }
    void set_forward_button(button::Button *bttn) { forwards_button_ = bttn; }
    void set_hw_reset_button(button::Button *bttn) { hw_reset_button_ = bttn; }
    void set_reset_pose_button(button::Button *bttn) { reset_pose_button_ = bttn; }
    void center_button_action();
    void cancel_button_action();
    void ok_button_action();
    void forward_button_action();
    void backwards_button_action();
    void left_button_action();
    void right_button_action();
 

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
    void rotate(const float angle);    // angle [deg]
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
    void set_leds();  

    void set_servo_positions(const uint8_t a_position, const uint8_t b_position);
    void get_servo_positions(int & a_position, int & b_position);

    void set_behaviour(const uint8_t behaviour);
    void set_stm32_fw_compatible(bool compatible) { stm32_fw_compatible_ = compatible; };
    void push_alvik_command(char a) {alvik_command_list_.push_back(a);}

    void read_compass_data();

  protected:
    friend AlvikBatterySensor;
    int cycle_;

    uint8_t alvik_state_;
    uint8_t alvik_action_;

    float forward_move_distance_;
    float turn_degree_;

    GPIOPin *stm32_pin_{nullptr};
    GPIOPin *nano_pin_{nullptr};
    GPIOPin *reset_pin_{nullptr};
    GPIOPin *red_led_pin_{nullptr};
    GPIOPin *green_led_pin_{nullptr};
    GPIOPin *blue_led_pin_{nullptr};
    GPIOPin *i2c_switch1_pin_{nullptr};
    GPIOPin *i2c_switch2_pin_{nullptr};

    bool stm32_is_on_;
    bool stm32_fw_compatible_;

    // Any:    0b00000001;
    // OK:     0b00000010; o: OK
    // Cancel: 0b00000100; x: Cancel
    // Center: 0b00001000; c: Center
    // Up:     0b00010000; e: Forward (Elo"re)
    // Left:   0b00100000; b: Turn Left (Balra)
    // Down:   0b01000000; h: Backwards (Ha'tra)
    // Right   0b10000000; j: Turn Right (Jobbra)
    std::string alvik_command_list_;
    int last_command_time_;
    int last_sensor_time_;
    int last_command_received_time_;
    int sensor_group_;

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
    float yaw_est;
    float angle_at_offset;
    uint8_t move_bits;

    float imu[6];

    float compass_angle;
    float compass_measurements[3];
    float compass_x_min;
    float compass_x_max;
    float compass_y_min;
    float compass_y_max;
    float compass_z_min;
    float compass_z_max;
    float compass_x_offset;
    float compass_y_offset;
    float compass_z_offset;

    //[L, CL, C, CR, R, T, B] [mm]
    int16_t distances[7];

    uint8_t touch, touch_bits;

    float joints_velocity[2];

    //[Left, Right] degree
    float joints_position[2];

    float robot_velocity[2];

    float robot_pose[3];

    uint8_t battery_;

    //sensor::Sensor *battery_sensor_;
    AlvikBatterySensor *battery_sensor_;
    AlvikCompassSensor *compass_sensor_;
    sensor::Sensor *compass_x_;
    sensor::Sensor *compass_y_;
    sensor::Sensor *compass_z_;
    sensor::Sensor *alvik_alive_sensor_;
    sensor::Sensor *alvik_action_state_;
    sensor::Sensor *pose_x_sensor_;
    sensor::Sensor *pose_y_sensor_;
    sensor::Sensor *pose_ang_sensor_;
    sensor::Sensor *roll_sensor_;
    sensor::Sensor *pitch_sensor_;
    sensor::Sensor *yaw_sensor_;
    sensor::Sensor *yaw_est_sensor_;
    sensor::Sensor *joints_l_;
    sensor::Sensor *joints_r_;
    sensor::Sensor *distance_l_;
    sensor::Sensor *distance_cl_;
    sensor::Sensor *distance_c_;
    sensor::Sensor *distance_cr_;
    sensor::Sensor *distance_r_;
    sensor::Sensor *distance_t_;
    sensor::Sensor *distance_b_;


    number::Number *forward_distance_;
    number::Number *turn_degree_number_;

    button::Button *center_button_;
    button::Button *cancel_button_;
    button::Button *ok_button_;
    button::Button *turn_right_button_;
    button::Button *turn_left_button_;
    button::Button *backwards_button_;
    button::Button *forwards_button_;
    button::Button *hw_reset_button_;
    button::Button *reset_pose_button_;

    text_sensor::TextSensor *fw_version_sensor_;
    text_sensor::TextSensor *lib_version_sensor_;
    text_sensor::TextSensor *command_list_sensor_;
 
    switch_::Switch *enable_alvik_switch_;

    static void update_task_(void *param);

};

class AlvikEnableSwitch : public switch_::Switch, public Parented<AlvikComponent> {
 public:
  AlvikEnableSwitch() = default;

 protected:
  void write_state(bool state) override;
};

class AlvikForwardButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikForwardButton() = default;

 protected:
  void press_action() override;
};

class AlvikBackwardsdButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikBackwardsdButton() = default;

 protected:
  void press_action() override;
};

class AlvikTurnLeftButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikTurnLeftButton() = default;

 protected:
  void press_action() override;
};

class AlvikTurnRightButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikTurnRightButton() = default;

 protected:
  void press_action() override;
};

class AlvikOKButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikOKButton() = default;

 protected:
  void press_action() override;
};

class AlvikCancelButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikCancelButton() = default;

 protected:
  void press_action() override;
};

class AlvikCenterButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikCenterButton() = default;

 protected:
  void press_action() override;
};

class AlvikResetButton : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikResetButton() = default;

 protected:
  void press_action() override;
};

class AlvikResetPoseButton  : public button::Button, public Parented<AlvikComponent> {
 public:
  AlvikResetPoseButton () = default;

 protected:
  void press_action() override;
};

class AlvikForwardDistance : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikForwardDistance() = default;

 protected:
  void control(float value) override;
};

class AlvikTurnDegree : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikTurnDegree() = default;

 protected:
  void control(float value) override;
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
