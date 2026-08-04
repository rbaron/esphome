from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import display, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_LAMBDA

CONF_RESET_PIN = "reset_pin"
CONF_BUSY_PIN = "busy_pin"

CODEOWNERS = ["@rbaron"]
DEPENDENCIES = ["uart"]

gu128x64d7000_ns = cg.esphome_ns.namespace("gu128x64d7000")
GU128X64D7000 = gu128x64d7000_ns.class_(
    "GU128X64D7000", display.DisplayBuffer, uart.UARTDevice
)

TurnOnAction = gu128x64d7000_ns.class_("TurnOnAction", automation.Action)
TurnOffAction = gu128x64d7000_ns.class_("TurnOffAction", automation.Action)
IsPoweredCondition = gu128x64d7000_ns.class_("IsPoweredCondition", automation.Condition)

ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(GU128X64D7000),
    }
)

CONFIG_SCHEMA = (
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(GU128X64D7000),
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_BUSY_PIN): pins.gpio_input_pin_schema,
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await uart.register_uart_device(var, config)

    reset = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset))

    if CONF_BUSY_PIN in config:
        busy = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
        cg.add(var.set_busy_pin(busy))

    await display.register_display(var, config)

    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))


@automation.register_action("gu128x64d7000.turn_on", TurnOnAction, ACTION_SCHEMA)
@automation.register_action("gu128x64d7000.turn_off", TurnOffAction, ACTION_SCHEMA)
async def gu128x64d7000_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_condition(
    "gu128x64d7000.is_powered", IsPoweredCondition, ACTION_SCHEMA
)
async def gu128x64d7000_is_powered_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
