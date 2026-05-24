from esphome import pins
import esphome.codegen as cg
from esphome.components import display
import esphome.config_validation as cv
from esphome.const import (
    CONF_CLOCK_PIN,
    CONF_DATA_PIN,
    CONF_ID,
    CONF_LAMBDA,
    CONF_OE_PIN,
)

CODEOWNERS = ["@rbaron"]

CONF_LATCH_PIN = "latch_pin"
CONF_DP_PIN = "dp_pin"
CONF_GRID_PINS = "grid_pins"
CONF_SCAN_INTERVAL = "scan_interval"

ld8035_clock_ns = cg.esphome_ns.namespace("ld8035_clock")
LD8035Clock = ld8035_clock_ns.class_("LD8035Clock", cg.PollingComponent)
LD8035ClockRef = LD8035Clock.operator("ref")

NUM_DIGITS = 4

CONFIG_SCHEMA = cv.All(
    display.BASIC_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(LD8035Clock),
            cv.Required(CONF_DATA_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_CLOCK_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_LATCH_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_OE_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DP_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_GRID_PINS): cv.All(
                cv.ensure_list(pins.gpio_output_pin_schema),
                cv.Length(min=NUM_DIGITS, max=NUM_DIGITS),
            ),
            cv.Optional(
                CONF_SCAN_INTERVAL, default="2ms"
            ): cv.positive_time_period_microseconds,
        }
    ),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await display.register_display(var, config)

    cg.add(var.set_ser_pin(await cg.gpio_pin_expression(config[CONF_DATA_PIN])))
    cg.add(var.set_srclk_pin(await cg.gpio_pin_expression(config[CONF_CLOCK_PIN])))
    cg.add(var.set_rclk_pin(await cg.gpio_pin_expression(config[CONF_LATCH_PIN])))
    cg.add(var.set_dp_pin(await cg.gpio_pin_expression(config[CONF_DP_PIN])))

    if oe_pin_config := config.get(CONF_OE_PIN):
        cg.add(var.set_oe_pin(await cg.gpio_pin_expression(oe_pin_config)))

    for i, pin_config in enumerate(config[CONF_GRID_PINS]):
        cg.add(var.set_grid_pin(i, await cg.gpio_pin_expression(pin_config)))

    cg.add(var.set_scan_interval_us(config[CONF_SCAN_INTERVAL].total_microseconds))

    if CONF_LAMBDA in config:
        lambda_ = await cg.process_lambda(
            config[CONF_LAMBDA], [(LD8035ClockRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))
