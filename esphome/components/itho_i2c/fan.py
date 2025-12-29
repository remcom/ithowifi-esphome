import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan
from esphome.const import CONF_ID, CONF_OUTPUT_ID
from . import itho_i2c_ns, IthoI2CComponent

DEPENDENCIES = ["itho_i2c"]

IthoI2CFan = itho_i2c_ns.class_("IthoI2CFan", fan.Fan, cg.Component)

CONFIG_SCHEMA = fan.fan_schema(IthoI2CFan).extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.use_id(IthoI2CComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_OUTPUT_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await cg.register_component(var, config)
    await fan.register_fan(var, config)
