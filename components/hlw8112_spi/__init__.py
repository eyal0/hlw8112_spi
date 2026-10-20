from esphome import pins
import esphome.codegen as cg
from esphome.components import libretiny, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_FREQUENCY,
    CONF_ID,
    CONF_PHASE_ANGLE,
    CONF_POWER_FACTOR,
    CONF_VOLTAGE,
    DEVICE_CLASS_APPARENT_POWER,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_DEGREES,
    UNIT_HERTZ,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT,
    UNIT_VOLT_AMPS,
    UNIT_WATT,
)
from esphome.types import ConfigType

DEPENDENCIES = ["libretiny"]
AUTO_LOAD = ["sensor"]

CONF_CS_PIN = "cs_pin"
CONF_SPI_FREQUENCY = "spi_frequency"
CONF_CHIP_CLOCK = "chip_clock"
CONF_CURRENT_RATIO_A = "current_ratio_a"
CONF_CURRENT_RATIO_B = "current_ratio_b"
CONF_VOLTAGE_RATIO = "voltage_ratio"
CONF_METER_CHANNEL = "meter_channel"
CONF_CURRENT_A = "current_a"
CONF_CURRENT_B = "current_b"
CONF_POWER_A = "power_a"
CONF_POWER_B = "power_b"
CONF_APPARENT_POWER = "apparent_power"
CONF_ENERGY_A = "energy_a"
CONF_ENERGY_B = "energy_b"
CONF_ENERGY_EXPORT_A = "energy_export_a"
CONF_ENERGY_EXPORT_B = "energy_export_b"

hlw8112_spi_ns = cg.esphome_ns.namespace("hlw8112_spi")
HLW8112SPI = hlw8112_spi_ns.class_("HLW8112SPI", cg.PollingComponent)


def _measurement(unit: str, accuracy: int, device_class: str):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        accuracy_decimals=accuracy,
        device_class=device_class,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _energy():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    )


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HLW8112SPI),
            cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SPI_FREQUENCY, default=500000): cv.int_range(min=100000, max=900000),
            cv.Optional(CONF_CHIP_CLOCK, default=3579545): cv.int_range(min=1000000, max=6000000),
            cv.Optional(CONF_CURRENT_RATIO_A, default=0.2): cv.positive_float,
            cv.Optional(CONF_CURRENT_RATIO_B, default=0.2): cv.positive_float,
            cv.Optional(CONF_VOLTAGE_RATIO, default=1.0): cv.positive_float,
            cv.Optional(CONF_METER_CHANNEL, default="A"): cv.enum({"A": False, "B": True}, upper=True),
            cv.Optional(CONF_VOLTAGE): _measurement(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional(CONF_FREQUENCY): _measurement(UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY),
            cv.Optional(CONF_CURRENT_A): _measurement(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            cv.Optional(CONF_CURRENT_B): _measurement(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            cv.Optional(CONF_POWER_A): _measurement(UNIT_WATT, 1, DEVICE_CLASS_POWER),
            cv.Optional(CONF_POWER_B): _measurement(UNIT_WATT, 1, DEVICE_CLASS_POWER),
            cv.Optional(CONF_APPARENT_POWER): _measurement(UNIT_VOLT_AMPS, 1, DEVICE_CLASS_APPARENT_POWER),
            cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER_FACTOR,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PHASE_ANGLE): sensor.sensor_schema(
                unit_of_measurement=UNIT_DEGREES,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY_A): _energy(),
            cv.Optional(CONF_ENERGY_B): _energy(),
            cv.Optional(CONF_ENERGY_EXPORT_A): _energy(),
            cv.Optional(CONF_ENERGY_EXPORT_B): _energy(),
        }
    )
    .extend(cv.polling_component_schema("10s")),
    libretiny.only_on_family(
        supported=[libretiny.const.FAMILY_BK7238, libretiny.const.FAMILY_BK7231N]
    ),
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    cg.add(var.set_cs_pin(pin))
    cg.add(var.set_spi_frequency(config[CONF_SPI_FREQUENCY]))
    cg.add(var.set_chip_clock(config[CONF_CHIP_CLOCK]))
    cg.add(var.set_current_ratio_a(config[CONF_CURRENT_RATIO_A]))
    cg.add(var.set_current_ratio_b(config[CONF_CURRENT_RATIO_B]))
    cg.add(var.set_voltage_ratio(config[CONF_VOLTAGE_RATIO]))
    cg.add(var.set_channel_b(config[CONF_METER_CHANNEL]))

    for key, setter in (
        (CONF_VOLTAGE, var.set_voltage_sensor),
        (CONF_FREQUENCY, var.set_frequency_sensor),
        (CONF_CURRENT_A, var.set_current_a_sensor),
        (CONF_CURRENT_B, var.set_current_b_sensor),
        (CONF_POWER_A, var.set_power_a_sensor),
        (CONF_POWER_B, var.set_power_b_sensor),
        (CONF_APPARENT_POWER, var.set_apparent_power_sensor),
        (CONF_POWER_FACTOR, var.set_power_factor_sensor),
        (CONF_PHASE_ANGLE, var.set_phase_angle_sensor),
        (CONF_ENERGY_A, var.set_energy_a_sensor),
        (CONF_ENERGY_B, var.set_energy_b_sensor),
        (CONF_ENERGY_EXPORT_A, var.set_energy_export_a_sensor),
        (CONF_ENERGY_EXPORT_B, var.set_energy_export_b_sensor),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter(sens))
