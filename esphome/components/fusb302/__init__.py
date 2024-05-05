import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.automation import maybe_simple_id
from esphome import pins
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_INTERRUPT_PIN,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_TRIGGER_ID,
)

CONF_START_POWER_NEGOTIATION_ON_BOOT = "start_power_negotiation_on_boot"
CONF_ON_PD_NEGOTIATION_SUCCESS = "on_pd_negotiation_success"
CONF_ON_PD_NEGOTIATION_FAILURE = "on_pd_negotiation_failure"

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

CONF_I2C_ADDR = 0x22
CONF_VBUS_VOLTAGE = "vbus_voltage_sensor"

fusb302_ns = cg.esphome_ns.namespace("fusb302")
FUSB302 = fusb302_ns.class_("FUSB302", cg.Component, i2c.I2CDevice)

OnPDNegotiationSuccessTrigger = fusb302_ns.class_(
    "OnPDNegotiationSuccessTrigger", automation.Trigger.template(cg.bool_)
)

OnPDNegotiationFailureTrigger = fusb302_ns.class_(
    "OnPDNegotiationFailureTrigger", automation.Trigger.template(cg.bool_)
)

StartPowerNegotiationAction = fusb302_ns.class_(
    "StartPowerNegotiationAction", automation.Action
)

SetVoltageRequirementAction = fusb302_ns.class_(
    "SetVoltageRequirementAction", automation.Action
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FUSB302),
            cv.Required(CONF_VOLTAGE): cv.voltage,
            cv.Required(CONF_CURRENT): cv.current,
            cv.Optional(CONF_INTERRUPT_PIN): cv.All(
                pins.internal_gpio_input_pin_schema
            ),
            cv.Optional(CONF_START_POWER_NEGOTIATION_ON_BOOT, default=True): cv.boolean,
            cv.Optional(CONF_ON_PD_NEGOTIATION_SUCCESS): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        OnPDNegotiationSuccessTrigger
                    )
                }
            ),
            cv.Optional(CONF_ON_PD_NEGOTIATION_FAILURE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        OnPDNegotiationFailureTrigger
                    )
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(CONF_I2C_ADDR))
)

ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(FUSB302),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_voltage_requirement(1000 * config[CONF_VOLTAGE]))
    cg.add(var.set_current_requirement(1000 * config[CONF_CURRENT]))

    if CONF_INTERRUPT_PIN in config:
        interrupt_pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
        cg.add(var.set_interrupt_pin(interrupt_pin))

    cg.add(
        var.set_start_power_negotiation_on_boot(
            config[CONF_START_POWER_NEGOTIATION_ON_BOOT]
        )
    )

    if CONF_VBUS_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VBUS_VOLTAGE])
        cg.add(var.set_vbus_voltage_sensor(sens))

    for conf in config.get(CONF_ON_PD_NEGOTIATION_SUCCESS, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(bool, "success")], conf)

    for conf in config.get(CONF_ON_PD_NEGOTIATION_FAILURE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(bool, "success")], conf)


@automation.register_action(
    "fusb302.start_power_negotiation", StartPowerNegotiationAction, ACTION_SCHEMA
)
async def fusb302_start_power_negotiation_to_code(
    config, action_id, template_arg, args
):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "fusb302.set_voltage_requirement",
    SetVoltageRequirementAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(FUSB302),
            cv.Required(CONF_VOLTAGE): cv.templatable(cv.voltage),
        }
    ),
)
async def fusb302_set_voltage_requirement_to_code(
    config, action_id, template_arg, args
):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_VOLTAGE], args, cg.float_)
    cg.add(var.set_voltage(template_))
    return var
