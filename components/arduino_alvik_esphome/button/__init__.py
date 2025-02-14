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
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    alvik_ns,
)

AlvikForwardButton = alvik_ns.class_("AlvikForwardButton", switch.Switch, cg.Component)
AlvikBackwardsdButton = alvik_ns.class_("AlvikBackwardsdButton", switch.Switch, cg.Component)
AlvikTurnLeftButton = alvik_ns.class_("AlvikTurnLeftButton", switch.Switch, cg.Component)
AlvikTurnRightButton = alvik_ns.class_("AlvikTurnRightButton", switch.Switch, cg.Component)
AlvikOKButton = alvik_ns.class_("AlvikOkButton", switch.Switch, cg.Component)
AlvikCancelButton = alvik_ns.class_("AlvikCancelButton", switch.Switch, cg.Component)
AlvikCenterButton = alvik_ns.class_("AlvikCenterButton", switch.Switch, cg.Component)

CONF_FORWARD_BUTTON = "forward_button"
CONF_BACKWARDS_BUTTON = "backwards_button"
CONF_TURNLEFT_BUTTON = "turn_left_button"
CONF_TURNRIGHT_BUTTON = "turn_right_button"
CONF_OK_BUTTON = "ok_button"
CONF_CANCEL_BUTTON = "cancel_button"
CONF_CENTER_BUTTON = "center_button"

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_FORWARD_BUTTON): button.button_schema(
            AlvikForwardButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_BACKWARDS_BUTTON): button.button_schema(
            AlvikBackwardsdButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_TURNLEFT_BUTTON): button.button_schema(
            AlvikTurnLeftButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_TURNRIGHT_BUTTON): button.button_schema(
            AlvikTurnRightButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_OK_BUTTON): button.button_schema(
            AlvikOKButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_CANCEL_BUTTON): button.button_schema(
            AlvikCancelButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_CENTER_BUTTON): button.button_schema(
            AlvikCenterButton,
            device_class=DEVICE_CLASS_BUTTON ,
            entity_category=ENTITY_CATEGORY_CONFIG,
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
    if ok_config := config.get(CONF_TURNRIGHT_BUTTON):
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
