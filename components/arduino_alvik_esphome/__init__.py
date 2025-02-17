import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.components import switch
from esphome.components import button
from esphome.components import sensor
from esphome.components import text_sensor
from esphome.components import i2c
from esphome.components import uart
from esphome import pins

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
#DEPENDENCIES = ["i2c", "uart"]
DEPENDENCIES = ["uart"]
CODEOWNERS = ["pipacsba"]

CONF_ALVIK_ID = "alvik_id"
CONF_CHECK_STM_PIN = "check_stm32_pin"
CONF_CHECK_NANO_PIN = "check_nano_pin"
CONF_RESET_STM_PIN = "reset_stm32_pin"

alvik_ns = cg.esphome_ns.namespace("alvik")
#AlvikComponent = alvik_ns.class_("AlvikComponent", cg.Component, i2c.I2CDevice, uart.UARTDevice)
AlvikComponent = alvik_ns.class_("AlvikComponent", cg.Component, uart.UARTDevice)

ALVIK_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ALVIK_ID): cv.use_id(AlvikComponent),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AlvikComponent),
            cv.Required(CONF_CHECK_STM_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_CHECK_NANO_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_RESET_STM_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    #.extend(i2c.i2c_device_schema(0x36))
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
#    await i2c.register_i2c_device(var, config)
    await uart.register_uart_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_CHECK_STM_PIN])
    cg.add(var.set_check_stm32_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_CHECK_NANO_PIN])
    cg.add(var.set_check_nano_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_RESET_STM_PIN])
    cg.add(var.set_reset_stm32_pin(pin))
