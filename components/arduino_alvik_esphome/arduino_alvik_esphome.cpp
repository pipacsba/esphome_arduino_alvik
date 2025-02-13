
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
           
    }

    void AlvikComponent::loop() {
        uint32_t now = millis();
        bool ison = this->pin_->digital_read();
        if (ison & !this->stm32_is_on_)
        {
           this->set_stm32_state(ison);
           ESP_LOGCONFIG(TAG, "The STM32 is turned on!");
           this->set_cycle(0);
        }
        else if (ison & this->stm32_is_on_)
        {
            this->set_cycle(this->cycle_ + 1);
            if (read_message()){
              parse_message();
            }

            
            //this is were we can do something with the Alvik
        }
        else if  (!ison & this->stm32_is_on_)
        {
           this->set_stm32_state(ison);
           ESP_LOGCONFIG(TAG, "The STM32 is turned off!");            
        }
    }

bool AlvikComponent::read_message(){                                               //it is private
  while (this->available()){
    b = this->read();
    packeter->buffer.push(b);
    if (packeter->checkPayload()){
      return true;
    }
  }
  return false;
}

int AlvikComponent::parse_message(){                                               //it is private
  code = packeter->payloadTop();
  switch(code){
    // get ack code
    case 'x':
      if (waiting_ack == NO_ACK){
        packeter->unpacketC1B(code, last_ack);
        last_ack = 0x00;
      } else {
        packeter->unpacketC1B(code, last_ack);
      }
      break;


    // motion

    // get joints velocity in RPM
    case 'j':
      while (!xSemaphoreTake(joint_vel_semaphore, 5)){}
      packeter->unpacketC2F(code, joints_velocity[0], joints_velocity[1]);
      xSemaphoreGive(joint_vel_semaphore);
      break;

    // get joints position in degrees
    case 'w':
      while (!xSemaphoreTake(joint_pos_semaphore, 5)){}
      packeter->unpacketC2F(code, joints_position[0], joints_position[1]);
      xSemaphoreGive(joint_pos_semaphore);
      break;

    // get robot linear and angular velocities in mm/s and degrees/s
    case 'v':
      while (!xSemaphoreTake(robot_vel_semaphore, 5)){}
      packeter->unpacketC2F(code, robot_velocity[0], robot_velocity[1]);
      xSemaphoreGive(robot_vel_semaphore);
      break;

    // get robot pose in mm and degrees, x, y, theta
    case 'z':
      while (!xSemaphoreTake(robot_pos_semaphore, 5)){}
      packeter->unpacketC3F(code, robot_pose[0], robot_pose[1], robot_pose[2]);
      xSemaphoreGive(robot_pos_semaphore);
      break;


    // sensors

    // get line follower sensors, low is white - high is black: Left, Center, Right
    case 'l':
      while (!xSemaphoreTake(line_semaphore, 5)){}
      packeter->unpacketC3I(code, line_sensors[0], line_sensors[1], line_sensors[2]);
      xSemaphoreGive(line_semaphore);
      break;

    // get colors: red, green, blue
    case 'c':
      while (!xSemaphoreTake(color_semaphore, 5)){}
      packeter->unpacketC3I(code, color_sensor[0], color_sensor[1], color_sensor[2]);
      xSemaphoreGive(color_semaphore);
      break;
    
    // get orientation in deg: roll, pitch, yaw
    case 'q':
      while (!xSemaphoreTake(orientation_semaphore, 5)){}
      packeter->unpacketC3F(code, orientation[0], orientation[1], orientation[2]);
      xSemaphoreGive(orientation_semaphore);
      break;

    // get tilt and shake
    case 'm':
      packeter->unpacketC1B(code, move_bits);
      break;

    // get imu data in g and deg/s: aX, aY, aZ, gX, gY, gZ
    case 'i':
      while (!xSemaphoreTake(imu_semaphore, 5)){}
      packeter->unpacketC6F(code, imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
      xSemaphoreGive(imu_semaphore);
      break;       
    
    // get data from ToF in mm: L, CL, C, CR, R, B, T
    case 'f':
      while (!xSemaphoreTake(distance_semaphore, 5)){}
      packeter->unpacketC7I(code, distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6]);
      xSemaphoreGive(distance_semaphore);
      break;    

    // get data from touch pads: any, ok, delete, center, left, down, right, up
    case 't':
      while (!xSemaphoreTake(touch_semaphore, 5)){}
      packeter->unpacketC1B(code, touch);
      xSemaphoreGive(touch_semaphore);
      break;   
    
    // get fw_version: Up, Mid, Low
    case 0x7E:
      while (!xSemaphoreTake(version_semaphore, 5)){}
      packeter->unpacketC3B(code, fw_version[0], fw_version[1], fw_version[2]);
      xSemaphoreGive(version_semaphore);
      break;

    // get battery parcentage: state of charge
    case 'p':
      packeter->unpacketC1F(code, battery);
      battery_is_charging = (battery > 0) ? true : false;
      battery = abs(battery);
      break;

    // nothing is parsed, the command is newer to this library
    default:
      return -1;
  }
  return 0;
}

    void AlvikComponent::dump_config() {
      ESP_LOGCONFIG(TAG, "AlvikComponent  : something is done!");
    }

    void AlvikEnableSwitch::write_state(bool state)
    {
    }

}  // namespace alvik
}  // namespace esphome
