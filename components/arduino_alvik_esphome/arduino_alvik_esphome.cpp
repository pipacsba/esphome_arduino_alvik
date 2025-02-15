
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
        this->set_cycle(0);
//          uint8_t battery_regs[] = {0, 0};
//          if ((this->write(&BATTERY_REGISTER, 1, false) != i2c::ERROR_OK) || !this->read_bytes_raw(battery_regs, 2)) {
//                ESP_LOGE(TAG, "Unable to i2c battery data");
//                this->mark_failed();
//                return;
//          }
//          uint16_t battery_val = encode_uint16(battery_regs[1], battery_regs[0]);
//          float battery_soc = battery_val * 0.00390625;
//          ESP_LOGD(TAG, "Read battery data: %d, %0.1f", battery_val, battery_soc);
        last_ack = NO_ACK;
        waiting_ack = NO_ACK;
        
        fw_version[0] = 0;
        fw_version[1] = 0;
        fw_version[2] = 0;

        lib_version[0] = 1;
        lib_version[1] = 1;
        lib_version[2] = 0;
        
        led_state = 0;
        
        line_sensors[0] = 0;
        line_sensors[1] = 0;
        line_sensors[2] = 0;
        
        color_sensor[0] = 0;
        color_sensor[1] = 0;
        color_sensor[2] = 0;
        
        servo_positions[0] = 90;
        servo_positions[1] = 90;
        
        orientation[0] = 0.0;
        orientation[1] = 0.0;
        orientation[2] = 0.0;
        yaw_est = 0.0;
        
        move_bits = 0;
        
        imu[0] = 0.0;
        imu[1] = 0.0;
        imu[2] = 0.0;
        imu[3] = 0.0;
        imu[4] = 0.0;
        imu[5] = 0.0;
        
        distances[0] = 0.0;
        distances[1] = 0.0;
        distances[2] = 0.0;
        distances[3] = 0.0;
        distances[4] = 0.0;
        distances[5] = 0.0;
        distances[6] = 0.0;
        
        touch = 0;
        touch_bits = 0;
        
        joints_velocity[0] = 0.0;
        joints_velocity[1] = 0.0;
      
        joints_position[0] = 0.0;
        joints_position[1] = 0.0;
       
        robot_velocity[0] = 0.0;
        robot_velocity[1] = 0.0;
        
        robot_pose[0] = 0.0;
        robot_pose[1] = 0.0;
        robot_pose[2] = 0.0;
        
        battery = 0.0;
        battery_soc = 0.0;
        battery_is_charging = false;

        this->set_stm32_fw_compatible(false);
        //this->stm_pin_->pin_mode(FLAG_PULLDOWN);
        this->nano_pin_->digital_write(false);
        this->reset_pin_->digital_write(false);
        
        this->flush();
        while (this->available()){
            this->read();
        }

        this->alvik_state_ = ALVIK_STARTUP;

        this->last_command_time_ = 0;
        this->last_sensor_time_  = 0;
        this->last_command_received_time_ = 0;
        this->alvik_command_list_.clear();
        this->alvik_action_= ACTION_PERFORM_COMMAND_LIST;
        
        ESP_LOGD(TAG, "Setup is finished, STM32 is in reset");
    }

    void AlvikComponent::loop() {
        uint32_t now = millis();
        uint8_t current_action;
        bool ison = this->stm32_pin_->digital_read();
        if ((!ison) & (this->alvik_state_ > ALVIK_HW_RESET))
        {
            this->alvik_state_ = ALVIK_STARTUP;
            this->cycle_ = 0;
        }
        switch(this->alvik_state_)
        {
            case ALVIK_STARTUP:
                {
                    if (this->cycle_ == 0)
                    {    
                        this->reset_pin_->digital_write(false);
                        this->nano_pin_->digital_write(true);
                        this->flush();
                        while (this->available()){
                            this->read();
                        }
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    this->cycle_ = this->cycle_ + 1;
                    if (this->cycle_ == 500)
                    {
                        this->reset_pin_->digital_write(true);
                        this->nano_pin_->digital_write(false);
                        this->alvik_state_ = ALVIK_HW_RESET;
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    break;
                }
            case ALVIK_HW_RESET:
                {
                    if (ison)
                    {
                        this->set_stm32_state(ison);
                        ESP_LOGD(TAG, "STM32 is up again");
                        this->set_cycle(0);
                        this->waiting_ack = 0x00;
                        this->alvik_state_ = ALVIK_STM32_UP;
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    break;
                }
            case ALVIK_STM32_UP:
                {
                    if (read_message())
                    {
                        parse_message();
                    }
                    if (this->last_ack == this->waiting_ack)
                    {
                        this->alvik_state_ = ALVIK_FIRST_ACK;
                        ESP_LOGD(TAG, "Wait_for_Ack completed!");
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    break;
                }
            case ALVIK_FIRST_ACK:
                {
                    if (read_message())
                    {
                        parse_message();
                    }
                    if (this->stm32_fw_compatible_)
                    {
                        this->alvik_state_ = ALVIK_FW_COMPATIBLE;
                        this->set_behaviour(BEHAVIOUR_ILLUMINATOR_RISE);
                        this->set_behaviour(BEHAVIOUR_BATTERY_ALERT);
                        this->set_servo_positions(0,0);
                        this->yaw_est = this->orientation[2];
                        this->alvik_action_= ACTION_PERFORM_COMMAND_LIST;
                    }
                    break;
                }
            case ALVIK_FW_COMPATIBLE:
                {
                    this->cycle_ = this->cycle_ + 1;
                    current_action = this->cycle_ % 3;
                    switch (current_action)
                    {
                        case TASK_READ_UART:
                            if (read_message())
                            {
                                parse_message();
                            }
                            if (read_message())
                            {
                                parse_message();
                            }
                            break;
                        case TASK_PERFORM_ACTION:
                            if (this->alvik_action_ == ACTION_PERFORM_COMMAND_LIST)
                            {
                                this->do_one_item_from_command_list(now);
                            }
                            if (this->alvik_action_ == ACTION_COLLECT_COMMAND_LIST)
                            {
                                if ((now - this->last_command_received_time_) < 50)
                                    this->change_alvik_left_right_leds(LEFT_BLUE + RIGHT_BLUE, true);
                                if ((now - this->last_command_received_time_) > 300)
                                    this->change_alvik_left_right_leds(LEFT_BLUE + RIGHT_BLUE, false);
                            }
                            break;
                        case TASK_WRITE_SENSOR:
                            if ((now - this->last_sensor_time_) >= 1 * 1000)
                            {
                                if (this->battery_sensor_ != nullptr)
                                    this->battery_sensor_->publish_state(this->battery);
                                if (this->alvik_alive_sensor_ != nullptr)
                                    this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                                if (this->pose_x_sensor_ != nullptr)
                                    this->pose_x_sensor_->publish_state(this->robot_pose[0]);
                                if (this->pose_y_sensor_ != nullptr)
                                    this->pose_y_sensor_->publish_state(this->robot_pose[1]);
                                if (this->pose_ang_sensor_ != nullptr)
                                    this->pose_ang_sensor_->publish_state(this->robot_pose[2]);
                                if (this->command_list_sensor_ != nullptr)
                                    this->command_list_sensor_->publish_state(this->alvik_command_list_);
                                if (this->roll_sensor_ != nullptr)
                                    this->roll_sensor_->publish_state(this->orientation[0]);
                                if (this->pitch_sensor_ != nullptr)
                                    this->pitch_sensor_->publish_state(this->orientation[1]);
                                if (this->yaw_sensor_ != nullptr)
                                    this->yaw_sensor_->publish_state(this->orientation[2]);
                                if (this->yaw_est_sensor_ != nullptr)
                                    this->yaw_est_sensor_->publish_state(this->yaw_est);
                                
                                this->last_sensor_time_= now;
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                }
            default:
                break;
        }

    }

    void AlvikComponent::do_one_item_from_command_list(uint32_t now)
    {
        if ((this->alvik_command_list_.length() != 0 ) & ((now - this->last_command_time_) >= 3 * 1000) )
        {
            //do user requests
            char c = this->alvik_command_list_[0];
            if (c == 0x65) // e
            {
                this->move(150);
            }
            else if (c == 0x68) // h
            {
                this->move(-150);
            }
            else if (c == 0x6a) // j
            {
                this->rotate(-90);
                this->yaw_est -= 90;
                if (yaw_est < 0) this->yaw_est += 360;
            }
            else if (c == 0x62) // b
            {
                this->rotate(90);
                this->yaw_est += 90;
                if (yaw_est > 360) this->yaw_est -= 360;
            }
            //clear the fulfilled request
            if (this->alvik_command_list_.length() > 1)
            {
                this->alvik_command_list_ = this->alvik_command_list_.substr(1);
            }
            else
            {
                this->alvik_command_list_.clear();
                this->change_alvik_left_right_leds(0xff, false);
            }
            this->last_command_time_ = now;
        }
    }

    bool AlvikComponent::read_message(){                                               //it is private
      while (this->available()){
        this->b = this->read();
        this->packeter->buffer.push(b);
        if (this->packeter->checkPayload()){
            ESP_LOGVV(TAG, "Incoming Message found!"); 
            return true;
        }
      }
      return false;
    }
    
    int AlvikComponent::parse_message(){                                               //it is private
      this->code = this->packeter->payloadTop();
      switch(code){
        // get ack code
        case 'x':
          if (this->waiting_ack == NO_ACK)
          {
            this->packeter->unpacketC1B(this->code, last_ack);
            last_ack = 0x00;
          } 
          else 
          {
            this->packeter->unpacketC1B(this->code, last_ack);
          }
          ESP_LOGVV(TAG, "Acknowledgement recieved!");
          break;
       
        // motion
    
        // get joints velocity in RPM
        case 'j':
          this->packeter->unpacketC2F(this->code, joints_velocity[0], joints_velocity[1]);
          break;
    
        // get joints position in degrees
        case 'w':
          this->packeter->unpacketC2F(this->code, joints_position[0], joints_position[1]);
          break;
    
        // get robot linear and angular velocities in mm/s and degrees/s
        case 'v':
          this->packeter->unpacketC2F(this->code, robot_velocity[0], robot_velocity[1]);
          break;
    
        // get robot pose in mm and degrees, x, y, theta
        case 'z':
          this->packeter->unpacketC3F(this->code, robot_pose[0], robot_pose[1], robot_pose[2]);
          break;
    
    
        // sensors
    
        // get line follower sensors, low is white - high is black: Left, Center, Right
        case 'l':
          this->packeter->unpacketC3I(this->code, line_sensors[0], line_sensors[1], line_sensors[2]);
          break;
    
        // get colors: red, green, blue
        case 'c':
          this->packeter->unpacketC3I(this->code, color_sensor[0], color_sensor[1], color_sensor[2]);
          break;
        
        // get orientation in deg: roll, pitch, yaw
        case 'q':
          this->packeter->unpacketC3F(this->code, orientation[0], orientation[1], orientation[2]);
          break;
    
        // get tilt and shake
        case 'm':
          this->packeter->unpacketC1B(this->code, move_bits);
          break;
    
        // get imu data in g and deg/s: aX, aY, aZ, gX, gY, gZ
        case 'i':
          this->packeter->unpacketC6F(this->code, imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
          break;       
        
        // get data from ToF in mm: L, CL, C, CR, R, B, T
        case 'f':
          this->packeter->unpacketC7I(this->code, distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6]);
          break;    
    
        // get data from touch pads: any, ok, delete, center, left, down, right, up
        case 't':
            this->packeter->unpacketC1B(this->code, touch);
            if ((millis() - this->last_command_received_time_) >= 500)
            {
                // Any:    0b00000001;
                // OK:     0b00000010; o: OK
                // Cancel: 0b00000100; x: Cancel
                // Center: 0b00001000; c: Center
                // Up:     0b00010000; e: Forward (Elo"re)
                // Left:   0b00100000; b: Turn Left (Balra)
                // Down:   0b01000000; h: Backwards (Ha'tra)
                // Right   0b10000000; j: Turn Right (Jobbra)
                if (touch & 0b00000010)
                    this->ok_button_action();
                if (touch & 0b00000100)
                    this->cancel_button_action();
                if (touch & 0b00001000)
                    this->center_button_action();
                if (touch & 0b00010000)
                    this->forward_button_action();
                if (touch & 0b00100000)
                    this->left_button_action();
                if (touch & 0b01000000)
                    this->backwards_button_action();
                if (touch & 0b10000000)
                    this->right_button_action();
                if (touch & 0b00000001)
                    this->last_command_received_time_ = millis();
            }
            break;   
        
        // get fw_version: Up, Mid, Low
        case 0x7E:
          this->packeter->unpacketC3B(this->code, fw_version[0], fw_version[1], fw_version[2]);
          if (fw_version == lib_version)
              { this->set_stm32_fw_compatible(true); }
          else
              { this->set_stm32_fw_compatible(true); }
          break;
    
        // get battery parcentage: state of charge
        case 'p':
          this->packeter->unpacketC1F(this->code, battery);
          this->battery_is_charging = (battery > 0) ? true : false;
          battery = abs(battery);
          break;
    
        // nothing is parsed, the command is newer to this library
        default:
          return -1;
      }
      return 0;
    }

    void AlvikComponent::center_button_action()
    {
        //alvik_command_list_.push_back('c'); // c: Center
        if (this->alvik_action_ == ACTION_PERFORM_COMMAND_LIST)
        {
            this->alvik_action_= ACTION_COLLECT_COMMAND_LIST;
            this->change_alvik_left_right_leds(LEFT_GREEN + RIGHT_GREEN, true);
        }
    }
    void AlvikComponent::cancel_button_action()
    {
        //alvik_command_list_.push_back('x'); // x: Cancel
        this->alvik_command_list_.clear();
        this->change_alvik_left_right_leds(LEFT_RED + RIGHT_RED, true);
    }
    void AlvikComponent::ok_button_action()
    {
        //alvik_command_list_.push_back('o'); // o: OK
        this->alvik_action_= ACTION_PERFORM_COMMAND_LIST;
        this->led_state= 0;
        this->yaw_est = this->orientation[2];
        this->change_alvik_left_right_leds(LEFT_BLUE + RIGHT_BLUE, true);
    }
    void AlvikComponent::forward_button_action()
    {
        alvik_command_list_.push_back('e'); // e: Forward (Elo"re)
    }
    void AlvikComponent::backwards_button_action()
    {
        alvik_command_list_.push_back('h'); // h: Backwards (Ha'tra)
    }
    void AlvikComponent::left_button_action()
    {
        alvik_command_list_.push_back('b'); // b: Turn Left (Balra)
    }
    void AlvikComponent::right_button_action()
    {
        alvik_command_list_.push_back('j'); // j: Turn Right (Jobbra)
    }

    void AlvikComponent::move(const float distance){
        this->msg_size = this->packeter->packetC1F('G', distance);
        this->write_array(this->packeter->msg, this->msg_size);
        this->waiting_ack = 'M';
        ESP_LOGD(TAG, "Move message sent!");
    }

    void AlvikComponent::rotate(const float angle){
        this->msg_size = this->packeter->packetC1F('R', angle);
        this->write_array(this->packeter->msg, this->msg_size);
        this->waiting_ack = 'R';
        ESP_LOGD(TAG, "Rotate message sent!");
    }

    void AlvikComponent::set_servo_positions(const uint8_t a_position, const uint8_t b_position){
        servo_positions[0] = a_position;
        servo_positions[1] = b_position;
        this->msg_size = this->packeter->packetC2B('S', a_position, b_position);
        this->write_array(this->packeter->msg, this->msg_size);
        ESP_LOGD(TAG, "Servo positions set to [%d,%d]!", a_position, b_position);
    }

    void AlvikComponent::set_behaviour(const uint8_t behaviour){
      this->msg_size = this->packeter->packetC1B('B', behaviour);
      this->write_array(this->packeter->msg, this->msg_size);
    }

    void AlvikComponent::change_alvik_left_right_leds(uint8_t change_led_state, bool onoff)
    {
        uint8_t a_led_state;
        if (onoff)
        {
            a_led_state = this->led_state | change_led_state;
        }
        else
        {
            a_led_state = this->led_state & (~change_led_state);
        }
        if (a_led_state != this->led_state)
        {
            ESP_LOGD(TAG, "LEDs requested to %x, %x -> %x, onoff: %d", a_led_state, change_led_state, this->led_state, onoff);
            this->led_state = a_led_state;
            this->msg_size = this->packeter->packetC1B('L', this->led_state);
            this->write_array(this->packeter->msg, this->msg_size);
        }
    }

    void AlvikComponent::dump_config() {
        ESP_LOGCONFIG(TAG, "AlvikComponent  :");
        ESP_LOGCONFIG(TAG, "   current state  : %d", this->alvik_state_);
        switch (this->alvik_state_)
        {
            case ALVIK_STARTUP:
                ESP_LOGCONFIG(TAG, "       Alvik Nano started up");
                break;
            case ALVIK_HW_RESET:
                ESP_LOGCONFIG(TAG, "       STM32 is reset");
                break;
            case ALVIK_STM32_UP:
                ESP_LOGCONFIG(TAG, "       STM32 is on, communication not yet established");
                break;
            case ALVIK_FIRST_ACK:
                ESP_LOGCONFIG(TAG, "       STM32 is on, communication is set up");
                break;
            case ALVIK_FW_COMPATIBLE:
                ESP_LOGCONFIG(TAG, "       Everything is ready to use");
                break;
            default:
                ESP_LOGCONFIG(TAG, "       State is unknown");
                break;

        }
        if (this->battery_sensor_ != nullptr)
        {
            ESP_LOGCONFIG(TAG, "   Battery status is : %.0f", this->battery_sensor_->get_state());
        }
    }    

    void AlvikEnableSwitch::write_state(bool state) {}


    void AlvikForwardButton::press_action() { this->parent_->forward_button_action(); } 
    void AlvikBackwardsdButton::press_action() { this->parent_->backwards_button_action(); } 
    void AlvikTurnLeftButton::press_action() { this->parent_->left_button_action(); } 
    void AlvikTurnRightButton::press_action() { this->parent_->right_button_action(); } 
    void AlvikOKButton::press_action() { this->parent_->ok_button_action(); } 
    void AlvikCancelButton::press_action() { this->parent_->cancel_button_action(); } 
    void AlvikCenterButton::press_action() { this->parent_->center_button_action(); } 

    void AlvikResetButton::press_action() 
    { 
        this->parent_->set_alvik_state(ALVIK_STARTUP);
        this->parent_->set_cycle(0); 
    } 


}  // namespace alvik
}  // namespace esphome
