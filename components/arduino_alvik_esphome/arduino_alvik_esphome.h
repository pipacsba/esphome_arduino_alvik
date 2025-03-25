
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


// behaviours
#define BEHAVIOUR_DISABLED 0
#define BEHAVIOUR_ILLUMINATOR_RISE 1
#define BEHAVIOUR_BATTERY_ALERT 2

const uint8_t BATTERY_REGISTER       = 0x06;
const uint8_t CHARGE_THRESHOLD       = 97;

const float MOTOR_MAX_RPM = 70.0; // 70

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
const float PI                           = 3.14159265358979323846;  /* pi */

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
const uint8_t ACTION_FOLLOW               = 2;
const uint8_t ACTION_CONSTANT_DIRECTION   = 3;
const uint8_t ACTION_MAZE_SOLVER          = 4;
const uint8_t ACTION_NOT_SET              = 255;


// LEFT and RIGHT LED COLORS
const uint8_t INTERNAL_LED = 1;
const uint8_t ILLUMINATOR  = 2;
const uint8_t LEFT_RED     = 4;
const uint8_t LEFT_GREEN   = 8;
const uint8_t LEFT_BLUE    = 16;
const uint8_t RIGHT_RED    = 32;
const uint8_t RIGHT_GREEN  = 64;
const uint8_t RIGHT_BLUE   = 128;

//MAZE INTERSECTION
const uint8_t INTERSECTION_NONE  = 0;
const uint8_t INTERSECTION_LEFT  = 1;
const uint8_t INTERSECTION_RIGHT = 2;
const uint8_t INTERSECTION_DEAD_END = 3;
const uint8_t INTERSECTION_STRAIGHT = 4;

//MAZE CRWALING STATE (maze_crawling_state_)
const uint8_t CRAWLING_STRAIGHT     = 0;
const uint8_t CRAWLING_TURNING      = 1;
const uint8_t CRAWLING_INTERSECTION = 2;
const uint8_t CRAWLING_SOLVED = 3;

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


    //-------------------------------------ESPHOME
    void setup() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::DATA; }
    void loop() override;

    //-------------------------------------ALVIK ACTIONS
    void do_one_item_from_command_list(uint32_t now);
    void external_supply_measurement(bool ison);
    void alvik_follow_control();
    void alvik_constant_direction_control();
    void alvik_maze_solver();
    void maze_are_we_there_yet();
    void alvik_line_follower();

    void set_cycle(int a_cycle) { cycle_ = a_cycle; };

    void center_button_action();
    void cancel_button_action();
    void ok_button_action();
    void forward_button_action();
    void backwards_button_action();
    void left_button_action();
    void right_button_action();

    void set_alvik_action(int an_action);
    void push_alvik_command(char a) {alvik_command_list_.push_back(a);}
    void set_alvik_state(int a_state) { alvik_state_ = a_state; };


    //-------------------------------------Exposed to Home Assistant
    // pin config
    void set_check_stm32_pin(GPIOPin *pin) { stm32_pin_ = pin; };
    void set_check_nano_pin(GPIOPin *pin) { nano_pin_ = pin; };
    void set_reset_stm32_pin(GPIOPin *pin) { reset_pin_ = pin; };
    void set_red_led_pin(GPIOPin *pin) { red_led_pin_ = pin; };
    void set_green_led_pin(GPIOPin *pin) { green_led_pin_ = pin; };
    void set_blue_led_pin(GPIOPin *pin) { blue_led_pin_ = pin; };
    void set_i2c_switch1_pin(GPIOPin *pin) { i2c_switch1_pin_ = pin; };
    void set_i2c_switch2_pin(GPIOPin *pin) { i2c_switch2_pin_ = pin; };

    //set NUMBER values
    void set_forward_move_distance(float a_distance) { forward_move_distance_ = a_distance; }
    void set_turn_degree(float an_angle) { turn_degree_ = an_angle; }
    void set_follow_target(float a_distance) {follow_target_ = a_distance;}
    void set_follow_tolerance(float a_distance) {follow_tolerance_ = a_distance;}
    void set_follow_Kp(float a_gain) {follow_Kp_ = a_gain;}
    void set_follow_K_horizontal(float a_gain) {follow_K_horizontal_ = a_gain;}
    void set_constant_direction_gain(float a_gain) {constant_direction_gain_ = a_gain;}
    void set_constant_direction_target(float an_angle) {constant_direction_target_angle_ = an_angle;}
    void set_line_follower_p(float a_gain) {line_follower_p_ = a_gain;}
    void set_line_follower_i(float a_gain) {line_follower_i_ = a_gain;}
    void set_line_follower_d(float a_gain) {line_follower_d_ = a_gain;}
    void set_maze_crawling_speed(float an_rpm) {maze_crawling_speed_ = an_rpm;}

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
    void set_to_centoid_sensor(sensor::Sensor *sensor1) { tof_centoid_ = sensor1; }
    void set_joint_l_speed_sensor(sensor::Sensor *sensor1) { wheel_speed_left_ = sensor1; }
    void set_joint_r_speed_sensor(sensor::Sensor *sensor1) { wheel_speed_right_ = sensor1; }
    void set_follow_start_sensor(sensor::Sensor *sensor1) { follow_start_sensor_ = sensor1; }
    void set_direction_control_start_sensor(sensor::Sensor *sensor1) { direction_control_start_sensor_ = sensor1; }
    void set_maze_solver_start_sensor(sensor::Sensor *sensor1) { maze_solver_start_sensor_ = sensor1; }
    void set_received_messages_counter_sensor(sensor::Sensor *sensor1) { received_messages_counter_sensor_ = sensor1; }
    void set_maze_crawling_state_sensor(sensor::Sensor *sensor1) { maze_crawling_state_sensor_ = sensor1; }
    void set_line_sensor_left(sensor::Sensor *sensor1) { line_sensor_left_ = sensor1; }
    void set_line_sensor_center(sensor::Sensor *sensor1) { line_sensor_center_ = sensor1; }
    void set_line_sensor_right(sensor::Sensor *sensor1) { line_sensor_right_ = sensor1; }

    // NUMBERS
    void set_forward_distance_number(number::Number *a_number) { forward_distance_ = a_number; }
    void set_turn_degree_number(number::Number *a_number) { turn_degree_number_ = a_number; }
    void set_follow_distance_config(number::Number *a_number) { follow_distance_number_ = a_number; }
    void set_follow_tolerance_config(number::Number *a_number) { follow_tolerance_number_ = a_number; }
    void set_follow_gain_h_config(number::Number *a_number) { follow_gain_horizontal_number_ = a_number; }
    void set_follow_gain_f_config(number::Number *a_number) { follow_gain_front_number_ = a_number; }
    void set_constant_direction_gain_config(number::Number *a_number) { constant_direction_gain_number_ = a_number; }
    void set_constant_direction_target_config(number::Number *a_number) { constant_direction_target_number_ = a_number; }
    void set_linefollower_d_config(number::Number *a_number) { linefollower_d_number_ = a_number; }
    void set_linefollower_i_config(number::Number *a_number) { linefollower_i_number_ = a_number; }
    void set_linefollower_p_config(number::Number *a_number) { linefollower_p_number_ = a_number; }
    void set_maze_crawling_speed_config(number::Number *a_number) { maze_crawling_speed_number_ = a_number; }


    //TEXT SENSORS
    void set_fw_sensor(text_sensor::TextSensor *sensor1) { fw_version_sensor_ = sensor1; }
    void set_lib_sensor(text_sensor::TextSensor *sensor1) { lib_version_sensor_ = sensor1; }
    void set_command_list_sensor(text_sensor::TextSensor *sensor1) { command_list_sensor_ = sensor1; }
    void set_maze_descriptor_sensor(text_sensor::TextSensor *sensor1) { maze_descriptor_sensor_ = sensor1; }

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


    //-------------------------------------ALVIK CARRIER INTERFACES
    void set_wheels_speed(const float left, const float right);  //RPM

    void rotate(const float angle);    // angle [deg]
    void move(const float distance);   // distance [mm]
    void brake() {set_wheels_speed(0,0);}

    void set_servo_positions(const uint8_t a_position, const uint8_t b_position);
    void reset_pose(const float x = 0.0, const float y = 0.0, const float theta = 0.0);

    void set_behaviour(const uint8_t behaviour);

    void set_stm32_fw_compatible(bool compatible) { stm32_fw_compatible_ = compatible; };
    void change_alvik_left_right_leds(uint8_t change_led_state, bool onoff);
    void set_stm32_state(bool ison) { stm32_is_on_ = ison; }

    //-------------------------------------LSM303DLHC (magnetic compass) measurement
    void read_compass_data();

    //-------------------------------------NOT YET USED AND IMPLEMENTED
    //void get_servo_positions(int & a_position, int & b_position);
    //void get_line_sensors(int & left, int & center, int & right);
    //void get_orientation(float & roll, float & pitch, float & yaw);
    //void get_accelerations(float & x, float & y, float & z);
    //void get_gyros(float & x, float & y, float & z);
    //void get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz);
    //bool get_shake();
    //void get_distance(float & left, float & center_left, float & center, float & center_right, float & right);
    //float get_distance_top();
    //float get_distance_bottom();
    //bool get_touch_any();
    //bool get_touch_ok();
    //bool get_touch_cancel();
    //bool get_touch_center();
    //bool get_touch_up();
    //bool get_touch_left();
    //bool get_touch_down();
    //bool get_touch_right();
    //void set_builtin_led(const bool value);
    //void set_illuminator(const bool value);
    //void set_leds();  
    //void get_wheels_position(float & left, float & right);
    //void set_wheels_position(const float left, const float right);
    //void get_drive_speed(float & linear, float & angular);
    //void drive(const float linear, const float angular);
    //void get_pose(float & x, float & y, float & theta);
    //void get_wheels_speed(float & left, float & right);


  protected:
    friend AlvikBatterySensor;

    //-------------------------------------ACTION_PERFORM_COMMAND_LIST; ACTION_COLLECT_COMMAND_LIST
    float forward_move_distance_;
    float turn_degree_;

    //-------------------------------------ACTION_CONSTANT_DIRECTION
    bool direction_control_start_;
    float constant_direction_target_angle_;
    float constant_direction_tolerance_angle_;
    float constant_direction_gain_;


    //-------------------------------------ACTION_FOLLOW
    bool follow_start_;
    float centoid_filt;
    float follow_target_;
    float follow_Kp_;
    float follow_K_horizontal_;
    float follow_tolerance_;
    float centoid_tolerance_;
    float wheel_speeds[2];


    //-------------------------------------MAZE_SOLVER
    bool maze_solver_start_;
    float line_detection_threshold_;
    std::string maze_solution_;
    float maze_crawling_speed_;
    float maze_crawling_speed_max_;
    int intersection_dir_;
    int maze_crawling_state_;
    int maze_saved_cycle_counter_;
    int maze_intersection_counter_;
    float maze_left_turn_confidence;
    bool maze_left_turn_confidence_decreasing_;
    float maze_right_turn_confidence;
    float maze_dead_end_confidence;
    float maze_straight_continue_confidence_inverze_;
    float maze_turn_started_confidence;
    float maze_turn_start_yaw_;
    bool maze_solved_;

    float line_follower_p_;
    float line_follower_i_;
    float line_follower_d_;
    float line_follower_centoid_previous_;
    float line_follower_centoid_integral_;

    //-------------------------------------general variables
    //cycle counter
    int cycle_;

    uint8_t alvik_state_;
    uint8_t alvik_action_;

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
    int received_messages_count_;

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
    bool distances_updated;

    uint8_t touch, touch_bits;

    float joints_velocity[2];

    //[Left, Right] degree
    float joints_position[2];

    float robot_velocity[2];

    float robot_pose[3];

    uint8_t battery_;


    //-------------------------------------Exposed to Home assistant
    //SENSOR
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
    sensor::Sensor *tof_centoid_;
    sensor::Sensor *wheel_speed_left_;
    sensor::Sensor *wheel_speed_right_;
    sensor::Sensor *received_messages_counter_sensor_;
    sensor::Sensor *follow_start_sensor_;
    sensor::Sensor *direction_control_start_sensor_;
    sensor::Sensor *maze_solver_start_sensor_;
    sensor::Sensor *line_sensor_left_;
    sensor::Sensor *line_sensor_center_;
    sensor::Sensor *line_sensor_right_;
    sensor::Sensor *maze_crawling_state_sensor_;

    //NUMBER    
    number::Number *forward_distance_;
    number::Number *turn_degree_number_;
    number::Number *follow_distance_number_;
    number::Number *follow_tolerance_number_;
    number::Number *follow_gain_horizontal_number_;
    number::Number *follow_gain_front_number_;
    number::Number *constant_direction_gain_number_;
    number::Number *constant_direction_target_number_;
    number::Number *linefollower_d_number_;
    number::Number *linefollower_i_number_;
    number::Number *linefollower_p_number_;
    number::Number *maze_crawling_speed_number_;

    //BUTTON
    button::Button *center_button_;
    button::Button *cancel_button_;
    button::Button *ok_button_;
    button::Button *turn_right_button_;
    button::Button *turn_left_button_;
    button::Button *backwards_button_;
    button::Button *forwards_button_;
    button::Button *hw_reset_button_;
    button::Button *reset_pose_button_;

    //TEXT_SENSOR
    text_sensor::TextSensor *fw_version_sensor_;
    text_sensor::TextSensor *lib_version_sensor_;
    text_sensor::TextSensor *command_list_sensor_;
    text_sensor::TextSensor *maze_descriptor_sensor_;

    //SWITCH
    switch_::Switch *enable_alvik_switch_;

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

class AlvikFollowDistance : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikFollowDistance() = default;

 protected:
  void control(float value) override;
};

class AlvikFollowTolerance : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikFollowTolerance() = default;

 protected:
  void control(float value) override;
};

class AlvikFollowGainHorizontal : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikFollowGainHorizontal() = default;

 protected:
  void control(float value) override;
};

class AlvikFollowGainFront : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikFollowGainFront() = default;

 protected:
  void control(float value) override;
};

class AlvikConstantDirectionGain : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikConstantDirectionGain() = default;

 protected:
  void control(float value) override;
};

class AlvikConstantDirectionTarget : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikConstantDirectionTarget() = default;

 protected:
  void control(float value) override;
};


class AlvikLineFollowerP : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikLineFollowerP() = default;

 protected:
  void control(float value) override;
};

class AlvikLineFollowerI : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikLineFollowerI() = default;

 protected:
  void control(float value) override;
};

class AlvikLineFollowerD : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikLineFollowerD() = default;

 protected:
  void control(float value) override;
};

class AlvikMazeCrawlingSpeed : public number::Number, public Parented<AlvikComponent> {
 public:
  AlvikMazeCrawlingSpeed() = default;

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
