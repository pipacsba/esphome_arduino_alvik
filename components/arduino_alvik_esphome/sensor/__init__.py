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
    UNIT_REVOLUTIONS_PER_MINUTE ,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
    UNIT_DEGREES,
    UNIT_MICROTESLA,
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    alvik_ns,
)

CONF_COMPASS_SENSOR = "compass_direction"
CONF_COMPASS_X_SENSOR = "compass_x"
CONF_COMPASS_Y_SENSOR = "compass_y"
CONF_COMPASS_Z_SENSOR = "compass_z"
CONF_BATTERY_CHARGE_SENSOR = "battery_charge"
CONF_ALIVE_SENSOR = "alvik_alive"
CONF_ACTION_SENSOR = "alvik_action_state"
CONF_ALVIK_POSE_X_SENSOR = "alvik_x_pose"
CONF_ALVIK_POSE_Y_SENSOR = "alvik_y_pose"
CONF_ALVIK_POSE_ANG_SENSOR = "alvik_ang_pose"
CONF_ALVIK_ROLL_SENSOR = "alvik_roll"
CONF_ALVIK_PITCH_SENSOR = "alvik_pitch"
CONF_ALVIK_YAW_SENSOR = "alvik_yaw"
CONF_ALVIK_YAW_EST_SENSOR = "alvik_yaw_estimated"
CONF_DISTANCE_L_SENSOR = "distance_l"
CONF_DISTANCE_CL_SENSOR = "distance_cl"
CONF_DISTANCE_C_SENSOR = "distance_c"
CONF_DISTANCE_CR_SENSOR = "distance_cr"
CONF_DISTANCE_R_SENSOR = "distance_r"
CONF_DISTANCE_T_SENSOR = "distance_t"
CONF_DISTANCE_B_SENSOR = "distance_b"
CONF_JOINTS_L_SENSOR = "joints_l"
CONF_JOINTS_R_SENSOR = "joints_r"
CONF_JOINTS_L_SPEED_SENSOR = "joints_l_speed"
CONF_JOINTS_R_SPEED_SENSOR = "joints_r_speed"
CONF_TOF_CENTOID_SENSOR = "tof_centoid"
CONF_RECEIVED_MESSAGES_COUNTER = "received_messages_counter"
CONF_ALVIK_FOLLOW_START = "follow_start"
CONF_CONSTANT_DIRECTION_START = "constant_direction_start"


AlvikBatterySensor = alvik_ns.class_("AlvikBatterySensor", sensor.Sensor, cg.Component, i2c.I2CDevice)
AlvikCompassSensor = alvik_ns.class_("AlvikCompassSensor", sensor.Sensor, cg.Component, i2c.I2CDevice)

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_JOINTS_L_SPEED_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE ,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_JOINTS_R_SPEED_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE ,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TOF_CENTOID_SENSOR): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=5,
        ),
        cv.Optional(CONF_COMPASS_SENSOR): sensor.sensor_schema(
            AlvikCompassSensor,
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(i2c.i2c_device_schema(0x1e)),
        cv.Optional(CONF_COMPASS_X_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MICROTESLA,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_COMPASS_Y_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MICROTESLA,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_COMPASS_Z_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MICROTESLA,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_JOINTS_R_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_JOINTS_L_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE_B_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE_T_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE_R_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE_CR_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE_C_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE_CL_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),        
        cv.Optional(CONF_DISTANCE_L_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIMETER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_CHARGE_SENSOR): sensor.sensor_schema(
            AlvikBatterySensor,
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(i2c.i2c_device_schema(0x36)),
        cv.Optional(CONF_ALIVE_SENSOR): sensor.sensor_schema(
        ),
        cv.Optional(CONF_ACTION_SENSOR): sensor.sensor_schema(
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
        cv.Optional(CONF_RECEIVED_MESSAGES_COUNTER): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ALVIK_FOLLOW_START): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CONSTANT_DIRECTION_START): sensor.sensor_schema(
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if follow_start_config := config.get(CONF_ALVIK_FOLLOW_START):
        sens = await sensor.new_sensor(follow_start_config)
        cg.add(alvik_id.set_follow_start_sensor(sens))
    if direction_start_config := config.get(CONF_CONSTANT_DIRECTION_START):
        sens = await sensor.new_sensor(direction_start_config)
        cg.add(alvik_id.set_direction_control_start_sensor(sens))
    if recmes_count_config := config.get(CONF_RECEIVED_MESSAGES_COUNTER):
        sens = await sensor.new_sensor(recmes_count_config)
        cg.add(alvik_id.set_received_messages_counter_sensor(sens))
    if joint_l_speed_config := config.get(CONF_JOINTS_L_SPEED_SENSOR):
        sens = await sensor.new_sensor(joint_l_speed_config)
        cg.add(alvik_id.set_joint_l_speed_sensor(sens))
    if joint_r_speed_config := config.get(CONF_JOINTS_R_SPEED_SENSOR):
        sens = await sensor.new_sensor(joint_r_speed_config)
        cg.add(alvik_id.set_joint_r_speed_sensor(sens))
    if tof_centoid_config := config.get(CONF_TOF_CENTOID_SENSOR):
        sens = await sensor.new_sensor(tof_centoid_config)
        cg.add(alvik_id.set_to_centoid_sensor(sens))
    if compass_config := config.get(CONF_COMPASS_SENSOR):
        sens = await sensor.new_sensor(compass_config)
        await i2c.register_i2c_device(sens, compass_config)
        cg.add(alvik_id.set_compass_sensor(sens))
    if compass_x := config.get(CONF_COMPASS_X_SENSOR):
        sens = await sensor.new_sensor(compass_x)
        cg.add(alvik_id.set_compass_x_sensor(sens))
    if compass_y := config.get(CONF_COMPASS_Y_SENSOR):
        sens = await sensor.new_sensor(compass_y)
        cg.add(alvik_id.set_compass_y_sensor(sens))
    if compass_z := config.get(CONF_COMPASS_Z_SENSOR):
        sens = await sensor.new_sensor(compass_z)
        cg.add(alvik_id.set_compass_z_sensor(sens))
    if joints_config := config.get(CONF_JOINTS_R_SENSOR):
        sens = await sensor.new_sensor(joints_config)
        cg.add(alvik_id.set_joints_r_sensor(sens))
    if joints_config := config.get(CONF_JOINTS_L_SENSOR):
        sens = await sensor.new_sensor(joints_config)
        cg.add(alvik_id.set_joints_l_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_B_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_b_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_T_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_t_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_R_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_r_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_CR_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_cr_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_C_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_c_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_CL_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_cl_sensor(sens))
    if distance_config := config.get(CONF_DISTANCE_L_SENSOR):
        sens = await sensor.new_sensor(distance_config)
        cg.add(alvik_id.set_distance_l_sensor(sens))
    if battery_charge_sensor_config := config.get(CONF_BATTERY_CHARGE_SENSOR):
        sens = await sensor.new_sensor(battery_charge_sensor_config)
        await i2c.register_i2c_device(sens, battery_charge_sensor_config)
        cg.add(alvik_id.set_battery_sensor(sens))
    if alive_sensor_config := config.get(CONF_ALIVE_SENSOR):
        sens = await sensor.new_sensor(alive_sensor_config)
        cg.add(alvik_id.set_alive_sensor(sens))
    if alive_action_config := config.get(CONF_ACTION_SENSOR):
        sens = await sensor.new_sensor(alive_action_config)
        cg.add(alvik_id.set_action_sensor(sens))
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
