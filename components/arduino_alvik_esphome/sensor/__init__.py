import esphome.codegen as cg
import esphome.config_validation as cv
#from esphome.components import number
#from esphome.components import switch
#from esphome.components import button
from esphome.components import sensor
#from esphome.components import text_sensor
from esphome.components import i2c

from esphome.const import (
    CONF_ID, 
    CONF_NAME, 
    CONF_NUMBER, 
    CONF_ENTITY_CATEGORY, 
    ENTITY_CATEGORY_CONFIG, 
    CONF_INITIAL_VALUE,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_RESTORE_VALUE,
    CONF_STEP,
    DEVICE_CLASS_BATTERY,
    UNIT_PERCENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
    UNIT_DEGREES,
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    alvik_ns,
)

CONF_BATTERY_CHARGE_SENSOR = "battery_charge"
CONF_ALIVE_SENSOR = "alvik_alive"
CONF_ALVIK_POSE_X_SENSOR = "alvik_x_pose"
CONF_ALVIK_POSE_Y_SENSOR = "alvik_y_pose"
CONF_ALVIK_POSE_ANG_SENSOR = "alvik_ang_pose"
CONF_ALVIK_ROLL_SENSOR = "alvik_roll"
CONF_ALVIK_PITCH_SENSOR = "alvik_pitch"
CONF_ALVIK_YAW_SENSOR = "alvik_yaw"
CONF_ALVIK_YAW_EST_SENSOR = "alvik_yaw_estimated"

AlvikBatterySensor = alvik_ns.class_("AlvikBatterySensor", sensor.Sensor, cg.Component, i2c.I2CDevice)

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_BATTERY_CHARGE_SENSOR): sensor.sensor_schema(
            AlvikBatterySensor,
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(i2c.i2c_device_schema(0x36)),
        cv.Optional(CONF_ALIVE_SENSOR): sensor.sensor_schema(
        ),
        cv.Optional(CONF_ALVIK_POSE_X_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_POSE_Y_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_POSE_ANG_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_ROLL_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_PITCH_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_YAW_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_YAW_EST_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if battery_charge_sensor_config := config.get(CONF_BATTERY_CHARGE_SENSOR):
        sens = await sensor.new_sensor(battery_charge_sensor_config)
        await i2c.register_i2c_device(sens, battery_charge_sensor_config)
        cg.add(alvik_id.set_battery_sensor(sens))
    if alive_sensor_config := config.get(CONF_ALIVE_SENSOR):
        sens = await sensor.new_sensor(alive_sensor_config)
        cg.add(alvik_id.set_alive_sensor(sens))
    if pose_x_config := config.get(CONF_ALVIK_POSE_X_SENSOR):
        sens = await sensor.new_sensor(pose_x_config)
        cg.add(alvik_id.set_pose_x_sensor(sens))
    if pose_y_config := config.get(CONF_ALVIK_POSE_Y_SENSOR):
        sens = await sensor.new_sensor(pose_y_config)
        cg.add(alvik_id.set_pose_y_sensor(sens))
    if pose_ang_config := config.get(CONF_ALVIK_POSE_ANG_SENSOR):
        sens = await sensor.new_sensor(pose_ang_config)
        cg.add(alvik_id.set_pose_ang_sensor(sens))
    if roll_config := config.get(CONF_ALVIK_ROLL_SENSOR):
        sens = await sensor.new_sensor(roll_config)
        cg.add(alvik_id.set_roll_sensor(sens))
    if pitch_config := config.get(CONF_ALVIK_PITCH_SENSOR):
        sens = await sensor.new_sensor(pitch_config)
        cg.add(alvik_id.set_pitch_sensor(sens))
    if yaw_config := config.get(CONF_ALVIK_YAW_SENSOR):
        sens = await sensor.new_sensor(yaw_config)
        cg.add(alvik_id.set_yaw_sensor(sens))
    if yaw_est_config := config.get(CONF_ALVIK_YAW_EST_SENSOR):
        sens = await sensor.new_sensor(yaw_est_config)
        cg.add(alvik_id.set_yaw_est_sensor(sens))
