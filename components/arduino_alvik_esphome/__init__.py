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
)

MULTI_CONF = False; #can be True in future if I understand the consequences
AUTO_LOAD = ["number", "switch", "button", "sensor", "text_sensor"]
CODEOWNERS = ["pipacsba"]

CONF_ALVIK_ID = "alvik_id"

alvik_ns = cg.esphome_ns.namespace("alvik")
AlvikComponent = alvik_ns.class_("AlvikComponent", cg.Component)

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
    ).extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
