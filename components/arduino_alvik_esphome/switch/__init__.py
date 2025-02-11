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
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_SWITCH,
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    TASK_UPDATE_TYPES,
    alvik_ns,
)

AlvikEnableSwitch = alvik_ns.class_("AlvikEnableSwitch", switch.Switch, cg.Component)

CONF_ENABLE_ALVIK_INIT_SWITCH = "enable_alvik"

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_ENABLE_ALVIK_INIT_SWITCH): switch.switch_schema(
            AlvikEnableSwitch,
            device_class=DEVICE_CLASS_SWITCH,
        ),
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if CONF_ENABLE_ALVIK_INIT_SWITCH in config:
        sw = await switch.new_switch(config[CONF_ENABLE_ALVIK_INIT_SWITCH])
        await cg.register_parented(sw, mainboard)
        cg.add(mainboard.set_enable_alvik_switch(sw))

