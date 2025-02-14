
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

        this->flush();
        while (this->available()){
            this->read();
        }

        this->nano_pin_->digital_write(FALSE);

        this->alvik_state = 0;
        
        ESP_LOGD(TAG, "Setup is finished");
    }

    void AlvikComponent::loop() {
        uint32_t now = millis();
        bool ison = this->stm32_pin_->digital_read();
        if (ison & !this->stm32_is_on_)
        {
            this->set_stm32_state(ison);
            ESP_LOGD(TAG, "The STM32 is turned on!");
            this->set_cycle(0);
            this->waiting_ack = 0x00;
            this->alvik_state = 0;
        }
        else if (ison & this->stm32_is_on_)
        {
            if (read_message())
            {
                parse_message();
            }
            else
            {
                if (this->alvik_state >0)
                {
                    this->set_cycle(this->cycle_ + 1);
                    //ESP_LOGD(TAG, "Alvik cycle is %d", this->cycle_);
                    if (this->cycle_ == 1500)
                    {
                        this->set_servo_positions(0,0);
                    }
                    if (this->cycle_ == 2000)
                    {
                        this->move(100);
                    }
                }
                else
                {
                    if (this->last_ack == this->waiting_ack)
                    {
                        this->alvik_state = 1;
                        ESP_LOGD(TAG, "Wait_for_Ack completed!");
                    }
                }
                
            }
            
            //this is were we can do something with the Alvik
        }
        else if  (!ison & this->stm32_is_on_)
        {
           this->set_stm32_state(ison);
           ESP_LOGD(TAG, "The STM32 is turned off!");            
        }
    }

    bool AlvikComponent::read_message(){                                               //it is private
      while (this->available()){
        this->b = this->read();
        this->packeter->buffer.push(b);
        if (this->packeter->checkPayload()){
            ESP_LOGD(TAG, "Incoming Message found!"); 
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
          ESP_LOGD(TAG, "Acknowledgement recieved!");
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
          break;   
        
        // get fw_version: Up, Mid, Low
        case 0x7E:
          this->packeter->unpacketC3B(this->code, fw_version[0], fw_version[1], fw_version[2]);
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

    void AlvikComponent::move(const float distance){
      this->msg_size = this->packeter->packetC1F('G', distance);
      this->write_array(this->packeter->msg, this->msg_size);
      this->waiting_ack = 'M';
      ESP_LOGD(TAG, "Move message sent!");
    }

    void AlvikComponent::set_servo_positions(const uint8_t a_position, const uint8_t b_position){
      servo_positions[0] = a_position;
      servo_positions[1] = b_position;
      this->msg_size = this->packeter->packetC2B('S', a_position, b_position);
      this->write_array(this->packeter->msg, this->msg_size);
      ESP_LOGD(TAG, "Servo positions set to [%d,%d]!", a_position, b_position);
    }

    void AlvikComponent::dump_config() {
      ESP_LOGCONFIG(TAG, "AlvikComponent  : something is done!");
    }

    void AlvikEnableSwitch::write_state(bool state)
    {
    }

}  // namespace alvik
}  // namespace esphome
