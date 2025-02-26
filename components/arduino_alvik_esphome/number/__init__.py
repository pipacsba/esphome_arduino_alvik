import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number

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
    DEVICE_CLASS_BUTTON,
    CONF_MODE,
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    alvik_ns,
)

AlvikForwardDistance = alvik_ns.class_("AlvikForwardDistance", number.Number)
AlvikTurnDegree = alvik_ns.class_("AlvikTurnDegree", number.Number)

CONF_FORWARD_DISTANCE = "move_forward_distance"
CONF_TURN_DEGREE = "turn_degree"

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_FORWARD_DISTANCE): number.number_schema(
            AlvikForwardDistance,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",

        ),
        cv.Optional(CONF_TURN_DEGREE): number.number_schema(
            AlvikTurnDegree,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:angle-acute",
            unit_of_measurement="°",

        ),
      
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])


    if angle_turn_config := config.get(CONF_TURN_DEGREE):
        n = await number.new_number(
            angle_turn_config,
            min_value=0,
            max_value=180,
            step=1,
            mode="BOX",
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_turn_degree_number(n))
        #cg.add(n.traits.set_mode(1))

    if forward_distance__config := config.get(CONF_FORWARD_DISTANCE):
        n = await number.new_number(
            forward_distance__config,
            min_value=0,
            max_value=1000,
            step=1,
            mode="BOX",
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_forward_distance_number(n))
        #cg.add(n.traits.set_mode(1))
