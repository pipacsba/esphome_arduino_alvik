import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.components import switch
from esphome.components import button
from esphome.components import sensor
from esphome.components import text_sensor

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
)

MULTI_CONF = False; #can be True in future if I understand the consequences
AUTO_LOAD = ["number", "switch", "button", "sensor", "text_sensor"]
CODEOWNERS = ["pipacsba"]

CONF_ALVIK_ID = "alvik_id"
CONF_BATTERY_CHARGE_SENSOR = "battery_charge"
CONF_FW_VERSION_SENSOR = "firmware_version"
CONF_LIB_VERSION_SENSOR = "library_version"

alvik_ns = cg.esphome_ns.namespace("alvik")
AlvikComponent = alvik_ns.class_("AlvikComponent", cg.Component)

ALVIK_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ALVIK_ID): cv.use_id(AlvikComponent),
    }
)

ALVIK_COMPONENT_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_BATTERY_CHARGE_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_FW_VERSION_SENSOR): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_LIB_VERSION_SENSOR): text_sensor.text_sensor_schema(
        ),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AlvikComponent),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if battery_charge_sensor_config := config.get(CONF_BATTERY_CHARGE_SENSOR):
        sens = await sensor.new_sensor(battery_charge_sensor_config)
        cg.add(mainboard.get_battery(sens))

    if fw_version_config := config.get(CONF_FW_VERSION_SENSOR):
        tsens = await text_sensor.new_text_sensor(fw_version_config)
        cg.add(mainboard.get_fw_version(tsens))

    if lib_version_config := config.get(CONF_LIB_VERSION_SENSOR):
        tsens = await text_sensor.new_text_sensor(lib_version_config)
        cg.add(mainboard.get_fw_version(tsens))
