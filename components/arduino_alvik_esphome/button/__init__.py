import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

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
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    alvik_ns,
)

AlvikForwardButton = alvik_ns.class_("AlvikForwardButton", button.Button, cg.Component)
AlvikBackwardsdButton = alvik_ns.class_("AlvikBackwardsdButton", button.Button, cg.Component)
AlvikTurnLeftButton = alvik_ns.class_("AlvikTurnLeftButton", button.Button, cg.Component)
AlvikTurnRightButton = alvik_ns.class_("AlvikTurnRightButton", button.Button, cg.Component)
AlvikOKButton = alvik_ns.class_("AlvikOKButton", button.Button, cg.Component)
AlvikCancelButton = alvik_ns.class_("AlvikCancelButton", button.Button, cg.Component)
AlvikCenterButton = alvik_ns.class_("AlvikCenterButton", button.Button, cg.Component)

CONF_FORWARD_BUTTON = "move_forward"
CONF_BACKWARDS_BUTTON = "move_backwards"
CONF_TURNLEFT_BUTTON = "turn_left"
CONF_TURNRIGHT_BUTTON = "turn_right"
CONF_OK_BUTTON = "ok"
CONF_CANCEL_BUTTON = "cancel"
CONF_CENTER_BUTTON = "center"

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_FORWARD_BUTTON): button.button_schema(
            AlvikForwardButton,
        ),
        cv.Optional(CONF_BACKWARDS_BUTTON): button.button_schema(
            AlvikBackwardsdButton,
        ),
        cv.Optional(CONF_TURNLEFT_BUTTON): button.button_schema(
            AlvikTurnLeftButton,
        ),
        cv.Optional(CONF_TURNRIGHT_BUTTON): button.button_schema(
            AlvikTurnRightButton,
        ),
        cv.Optional(CONF_OK_BUTTON): button.button_schema(
            AlvikOKButton,
        ),
        cv.Optional(CONF_CANCEL_BUTTON): button.button_schema(
            AlvikCancelButton,
        ),
        cv.Optional(CONF_CENTER_BUTTON): button.button_schema(
            AlvikCenterButton,
        ),
      
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if center_config := config.get(CONF_CENTER_BUTTON):
        b = await button.new_button(center_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_center_button(b))
    if cancel_config := config.get(CONF_CANCEL_BUTTON):
        b = await button.new_button(cancel_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_cancel_button(b))
    if ok_config := config.get(CONF_OK_BUTTON):
        b = await button.new_button(ok_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_ok_button(b))
    if tr_config := config.get(CONF_TURNRIGHT_BUTTON):
        b = await button.new_button(tr_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_turnright_button(b))
    if tl_config := config.get(CONF_TURNLEFT_BUTTON):
        b = await button.new_button(tl_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_turnleft_button(b))
    if bw_config := config.get(CONF_BACKWARDS_BUTTON):
        b = await button.new_button(bw_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_backwards_button(b))
    if fw_config := config.get(CONF_FORWARD_BUTTON):
        b = await button.new_button(fw_config)
        await cg.register_parented(b, config[CONF_ALVIK_ID])
        cg.add(alvik_id.set_forward_button(b))
