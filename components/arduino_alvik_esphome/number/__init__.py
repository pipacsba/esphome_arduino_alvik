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
    UNIT_REVOLUTIONS_PER_MINUTE,
)

from .. import (
    CONF_ALVIK_ID,
    ALVIK_COMPONENT_SCHEMA,
    alvik_ns,
)

AlvikForwardDistance = alvik_ns.class_("AlvikForwardDistance", number.Number)
AlvikTurnDegree = alvik_ns.class_("AlvikTurnDegree", number.Number)
AlvikFollowDistance = alvik_ns.class_("AlvikFollowDistance", number.Number)
AlvikFollowTolerance = alvik_ns.class_("AlvikFollowTolerance", number.Number)
AlvikFollowGainHorizontal = alvik_ns.class_("AlvikFollowGainHorizontal", number.Number)
AlvikFollowGainFront = alvik_ns.class_("AlvikFollowGainFront", number.Number)
AlvikConstantDirectionGain = alvik_ns.class_("AlvikConstantDirectionGain", number.Number)
AlvikConstantDirectionTarget = alvik_ns.class_("AlvikConstantDirectionTarget", number.Number)
AlvikLineFollowerP = alvik_ns.class_("AlvikLineFollowerP", number.Number)
AlvikLineFollowerI = alvik_ns.class_("AlvikLineFollowerI", number.Number)
AlvikLineFollowerD = alvik_ns.class_("AlvikLineFollowerD", number.Number)
AlvikMazeCrawlingSpeed = alvik_ns.class_("AlvikMazeCrawlingSpeed", number.Number)

CONF_FORWARD_DISTANCE = "move_forward_distance"
CONF_TURN_DEGREE = "turn_degree"
CONF_FOLLOW_DISTANCE = "follow_distance"
CONF_FOLLOW_TOLERANCE = "follow_tolerance"
CONF_FOLLOW_GAIN_HORIZONTAL = "follow_gain_horizontal"
CONF_FOLLOW_GAIN_FRONT = "follow_gain_front"
CONF_CONSTANT_DIRECTION_GAIN = "constant_direction_gain"
CONF_CONSTANT_DIRECTION_TARGET = "constant_direction_target"
CONF_LINEFOLLOWER_P = "line_follower_p"
CONF_LINEFOLLOWER_I = "line_follower_i"
CONF_LINEFOLLOWER_D = "line_follower_d"
CONF_MAZE_CRAWLING_SPEED = "maze_crawling_speed"

CONFIG_SCHEMA = ALVIK_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_FOLLOW_DISTANCE): number.number_schema(
            AlvikFollowDistance,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",
        ),
        cv.Optional(CONF_FOLLOW_TOLERANCE): number.number_schema(
            AlvikFollowTolerance,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:map-marker-distance",
            unit_of_measurement="mm",
        ),
        cv.Optional(CONF_FOLLOW_GAIN_HORIZONTAL): number.number_schema(
            AlvikFollowGainHorizontal,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_FOLLOW_GAIN_FRONT): number.number_schema(
            AlvikFollowGainFront,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
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
        cv.Optional(CONF_CONSTANT_DIRECTION_GAIN): number.number_schema(
            AlvikConstantDirectionGain,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_CONSTANT_DIRECTION_TARGET): number.number_schema(
            AlvikConstantDirectionTarget,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:angle-acute",
            unit_of_measurement="°",
        ),
        cv.Optional(CONF_LINEFOLLOWER_P): number.number_schema(
            AlvikLineFollowerP,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_LINEFOLLOWER_I): number.number_schema(
            AlvikLineFollowerI,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_LINEFOLLOWER_D): number.number_schema(
            AlvikLineFollowerD,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_MAZE_CRAWLING_SPEED): number.number_schema(
            AlvikMazeCrawlingSpeed,
            unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE 
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
    }
)

async def to_code(config):
    alvik_id = await cg.get_variable(config[CONF_ALVIK_ID])

    if mazecrawlingspeed_config := config.get(CONF_MAZE_CRAWLING_SPEED):
        n = await number.new_number(
            mazecrawlingspeed_config,
            min_value=0,
            max_value=100,
            step=1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_maze_crawling_speed_config(n))
    if linefollower_pid_config := config.get(CONF_LINEFOLLOWER_D):
        n = await number.new_number(
            linefollower_pid_config,
            min_value=0,
            max_value=100,
            step=0.5,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_linefollower_d_config(n))
    if linefollower_pid_config := config.get(CONF_LINEFOLLOWER_I):
        n = await number.new_number(
            linefollower_pid_config,
            min_value=0,
            max_value=20,
            step=0.5,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_linefollower_i_config(n))
    if linefollower_pid_config := config.get(CONF_LINEFOLLOWER_P):
        n = await number.new_number(
            linefollower_pid_config,
            min_value=0,
            max_value=100,
            step=0.5,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_linefollower_p_config(n))
    if constant_dir_target_config := config.get(CONF_CONSTANT_DIRECTION_TARGET):
        n = await number.new_number(
            constant_dir_target_config,
            min_value=0,
            max_value=360,
            step=1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_constant_direction_target_config(n))
    if constant_dir_gain_config := config.get(CONF_CONSTANT_DIRECTION_GAIN):
        n = await number.new_number(
            constant_dir_gain_config,
            min_value=0,
            max_value=10,
            step=0.5,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_constant_direction_gain_config(n))
    if follow_distance_config := config.get(CONF_FOLLOW_DISTANCE):
        n = await number.new_number(
            follow_distance_config,
            min_value=0,
            max_value=500,
            step=1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_follow_distance_config(n))
    if follow_tolerance_config := config.get(CONF_FOLLOW_TOLERANCE):
        n = await number.new_number(
            follow_tolerance_config,
            min_value=0,
            max_value=200,
            step=1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_follow_tolerance_config(n))
    if follow_gain_h_config := config.get(CONF_FOLLOW_GAIN_HORIZONTAL):
        n = await number.new_number(
            follow_gain_h_config,
            min_value=0,
            max_value=200,
            step=0.1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_follow_gain_h_config(n))
    if follow_gain_f_config := config.get(CONF_FOLLOW_GAIN_FRONT):
        n = await number.new_number(
            follow_gain_f_config,
            min_value=0,
            max_value=200,
            step=0.1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_follow_gain_f_config(n))
    if angle_turn_config := config.get(CONF_TURN_DEGREE):
        n = await number.new_number(
            angle_turn_config,
            min_value=0,
            max_value=180,
            step=1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_turn_degree_number(n))
    if forward_distance__config := config.get(CONF_FORWARD_DISTANCE):
        n = await number.new_number(
            forward_distance__config,
            min_value=0,
            max_value=1000,
            step=1,
        )
        await cg.register_parented(n, alvik_id)
        cg.add(alvik_id.set_forward_distance_number(n))
