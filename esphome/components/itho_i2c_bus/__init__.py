import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_SDA = "sda"
CONF_SCL = "scl"
CONF_FREQUENCY = "frequency"

itho_i2c_ns = cg.esphome_ns.namespace("itho_i2c")
IthoI2CBus = itho_i2c_ns.class_("IthoI2CBus", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(IthoI2CBus),
        cv.Required(CONF_SDA): cv.int_range(min=0, max=39),
        cv.Required(CONF_SCL): cv.int_range(min=0, max=39),
        cv.Optional(CONF_FREQUENCY, default=100000): cv.int_range(min=1000, max=400000),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sda_pin(config[CONF_SDA]))
    cg.add(var.set_scl_pin(config[CONF_SCL]))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
