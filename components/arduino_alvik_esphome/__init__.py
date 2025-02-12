import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.components import switch
from esphome.components import button
from esphome.components import sensor
from esphome.components import text_sensor
from esphome.components import i2c
from esphome.components import uart

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
)

MULTI_CONF = False; #can be True in future if I understand the consequences
AUTO_LOAD = ["number", "switch", "button", "sensor", "text_sensor"]
DEPENDENCIES = ["i2c", "uart"]
CODEOWNERS = ["pipacsba"]

CONF_ALVIK_ID = "alvik_id"

alvik_ns = cg.esphome_ns.namespace("alvik")
AlvikComponent = alvik_ns.class_("AlvikComponent", cg.Component, i2c.I2CDevice)

TaskUpdateType = alvik_ns.enum("TaskUpdateType")
TASK_UPDATE_TYPES = {
    "ENABLE_ALVIK" : TaskUpdateType.ENABLE_ALVIK,
}



ALVIK_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ALVIK_ID): cv.use_id(AlvikComponent),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AlvikComponent),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36))
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    await uart.register_uart_device(var, config)
