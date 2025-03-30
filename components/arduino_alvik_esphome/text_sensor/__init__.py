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

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
)

CONF_FW_VERSION_SENSOR = "firmware_version"
CONF_LIB_VERSION_SENSOR = "library_version"
CONF_COMMAND_LIST_SENSOR = "command_list"
CONF_MAZE_MAP_DESCRIPTOR_SENSOR = "maze_descriptor_sensor"
CONF_MAZE_KNOWN_MAP_DESCRIPTOR_SENSOR = "maze_solution_descriptor_sensor"


CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_FW_VERSION_SENSOR): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_LIB_VERSION_SENSOR): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_COMMAND_LIST_SENSOR): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_MAZE_MAP_DESCRIPTOR_SENSOR): text_sensor.text_sensor_schema(
        ),
        cv.Optional(CONF_MAZE_KNOWN_MAP_DESCRIPTOR_SENSOR): text_sensor.text_sensor_schema(
        ),
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if fw_version_config := config.get(CONF_FW_VERSION_SENSOR):
        tsens = await text_sensor.new_text_sensor(fw_version_config)
        cg.add(alvik_id.set_fw_sensor(tsens))

    if lib_version_config := config.get(CONF_LIB_VERSION_SENSOR):
        tsens = await text_sensor.new_text_sensor(lib_version_config)
        cg.add(alvik_id.set_lib_sensor(tsens))

    if lib_version_config := config.get(CONF_COMMAND_LIST_SENSOR):
        tsens = await text_sensor.new_text_sensor(lib_version_config)
        cg.add(alvik_id.set_command_list_sensor(tsens))

    if maze_descriptor_config := config.get(CONF_MAZE_MAP_DESCRIPTOR_SENSOR):
        tsens = await text_sensor.new_text_sensor(maze_descriptor_config)
        cg.add(alvik_id.set_maze_descriptor_sensor(tsens))

    if maze_descriptor_config := config.get(CONF_MAZE_KNOWN_MAP_DESCRIPTOR_SENSOR):
        tsens = await text_sensor.new_text_sensor(maze_descriptor_config)
        cg.add(alvik_id.set_maze_solution_descriptor_sensor(tsens))
