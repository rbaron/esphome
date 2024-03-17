import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome import pins
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_INTERRUPT_PIN,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_TRIGGER_ID,
)

CONF_ON_PD_NEGOTIATION_OUTCOME = "on_pd_negotiation_outcome"

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

CONF_I2C_ADDR = 0x22
CONF_VBUS_VOLTAGE = "vbus_voltage_sensor"

fusb302_ns = cg.esphome_ns.namespace("fusb302")
FUSB302 = fusb302_ns.class_("FUSB302", cg.Component, i2c.I2CDevice)

OnPDNegotiationTrigger = fusb302_ns.class_(
    "OnPDNegotiationTrigger", automation.Trigger.template(cg.bool_)
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FUSB302),
            cv.Required(CONF_INTERRUPT_PIN): cv.All(
                pins.internal_gpio_input_pin_schema
            ),
            cv.Required(CONF_VOLTAGE): cv.voltage,
            cv.Required(CONF_CURRENT): cv.current,
            cv.Optional(CONF_ON_PD_NEGOTIATION_OUTCOME): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnPDNegotiationTrigger)}
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(CONF_I2C_ADDR))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    interrupt_pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
    cg.add(var.set_interrupt_pin(interrupt_pin))
    cg.add(
        var.set_power_requirement(
            1000 * config[CONF_VOLTAGE], 1000 * config[CONF_CURRENT]
        )
    )
    # cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    if CONF_VBUS_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VBUS_VOLTAGE])
        cg.add(var.set_vbus_voltage_sensor(sens))

    for conf in config.get(CONF_ON_PD_NEGOTIATION_OUTCOME, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(bool, "success")], conf)
