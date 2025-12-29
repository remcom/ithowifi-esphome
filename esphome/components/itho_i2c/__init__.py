import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    CONF_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

DEPENDENCIES = []
MULTI_CONF = False
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_FAN_SPEED = "fan_speed"
CONF_FAN_SETPOINT = "fan_setpoint"
CONF_FAN_SPEED_RPM = "fan_speed_rpm"
CONF_DEVICE_TYPE = "device_type"
CONF_REMOTE_ID = "remote_id"
CONF_BUS_ID = "bus_id"
CONF_SDA = "sda"
CONF_SCL = "scl"
CONF_FREQUENCY = "frequency"
UNIT_RPM = "rpm"

itho_i2c_ns = cg.esphome_ns.namespace("itho_i2c")
IthoI2CBus = itho_i2c_ns.class_("IthoI2CBus", cg.Component)
IthoI2CComponent = itho_i2c_ns.class_("IthoI2CComponent", cg.Component)

# I2C Bus Schema
ITHO_I2C_BUS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(IthoI2CBus),
        cv.Required(CONF_SDA): cv.int_range(min=0, max=39),
        cv.Required(CONF_SCL): cv.int_range(min=0, max=39),
        cv.Optional(CONF_FREQUENCY, default=100000): cv.int_range(min=1000, max=400000),
    }
).extend(cv.COMPONENT_SCHEMA)

# Component Schema
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IthoI2CComponent),
            cv.GenerateID(CONF_BUS_ID): cv.use_id(IthoI2CBus),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FAN_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FAN_SETPOINT): sensor.sensor_schema(
                unit_of_measurement=UNIT_RPM,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FAN_SPEED_RPM): sensor.sensor_schema(
                unit_of_measurement=UNIT_RPM,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_DEVICE_TYPE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_REMOTE_ID): cv.All(
                cv.ensure_list(cv.hex_uint8_t), cv.Length(min=3, max=3)
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Link to the bus
    bus = await cg.get_variable(config[CONF_BUS_ID])
    cg.add(var.set_bus(bus))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))

    if CONF_FAN_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_FAN_SPEED])
        cg.add(var.set_fan_speed_sensor(sens))

    if CONF_FAN_SETPOINT in config:
        sens = await sensor.new_sensor(config[CONF_FAN_SETPOINT])
        cg.add(var.set_fan_setpoint_sensor(sens))

    if CONF_FAN_SPEED_RPM in config:
        sens = await sensor.new_sensor(config[CONF_FAN_SPEED_RPM])
        cg.add(var.set_fan_speed_rpm_sensor(sens))

    if CONF_DEVICE_TYPE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DEVICE_TYPE])
        cg.add(var.set_device_type_sensor(sens))

    if CONF_REMOTE_ID in config:
        remote_id = config[CONF_REMOTE_ID]
        cg.add(var.set_remote_id(remote_id[0], remote_id[1], remote_id[2]))


# Register the bus configuration
async def register_itho_i2c_bus(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_sda_pin(config[CONF_SDA]))
    cg.add(var.set_scl_pin(config[CONF_SCL]))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))

    return var
