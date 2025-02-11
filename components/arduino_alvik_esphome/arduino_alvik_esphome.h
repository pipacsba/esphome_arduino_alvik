
#pragma once

#ifndef __ALVIK_ESPHOME__
#define __ALVIK_ESPHOME__

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "Arduino_Alvik.h"
#include <vector>

namespace esphome {
namespace alvik {

   enum TaskUpdateType
    {
      SENSORS = 0,
      ENABLE_ALVIK,
    };

    typedef struct
    {
      TaskUpdateType type;
      union
      {
        bool b;
        int32_t i;
        float f;
        uint32_t u;
      } data;
    } TaskUpdate;

    class AlvikUpdateable
    {
    public:
      AlvikUpdateable() = default;
      void set_update_type(TaskUpdateType type) { type_ = type; }
    protected:
      TaskUpdateType type_;
    };

class AlvikComponent  : public Component {
  public:

    void setup() override;
    void dump_config() override;
  
    void loop() override;
  
    void set_battery_sensor(sensor::Sensor *sensor1) { battery_sensor_ = sensor1; }
    void set_alive_sensor(sensor::Sensor *sensor1) { alvik_alive_sensor_ = sensor1; }
    void set_fw_sensor(text_sensor::TextSensor *sensor1) { fw_version_sensor_ = sensor1; }
    void set_lib_sensor(text_sensor::TextSensor *sensor1) { lib_version_sensor_ = sensor1; }
    void set_enable_alvik_switch(switch_::Switch *sw) { enable_alvik_switch_ = sw; }

    void send_task_update(TaskUpdate update);

  protected:
    static const size_t UPDATE_TASK_STACK_SIZE_ = 3192;
    static const size_t UPDATE_TASK_QUEUE_SIZE_ = 10;
    static const uint32_t UPDATE_TASK_SENSOR_UPDATE_MS_ = 150;

    Arduino_Alvik alvik;
    uint8_t battery_;

    sensor::Sensor *battery_sensor_;
    sensor::Sensor *alvik_alive_sensor_;

    text_sensor::TextSensor *fw_version_sensor_;
    text_sensor::TextSensor *lib_version_sensor_;
 
    switch_::Switch *enable_alvik_switch_;

    static void update_task_(void *param);

    QueueHandle_t update_task_queue_ = NULL;

};

class AlvikSwitch : public switch_::Switch, public Parented<AlvikComponent>, public AlvikUpdateable
{
  public:
    AlvikSwitch() = default;

  protected:
    void write_state(bool state) override
    {
        TaskUpdate update;
        update.type = type_;
        update.data.b = state;
        this->parent_->send_task_update(update);
        this->publish_state(state);
    }
};

 
}  // namespace alvik
}  // namespace esphome

#endif
