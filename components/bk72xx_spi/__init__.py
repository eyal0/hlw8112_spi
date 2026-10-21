import esphome.codegen as cg
from esphome.components import libretiny
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["libretiny"]

bk72xx_spi_ns = cg.esphome_ns.namespace("bk72xx_spi")
BK72xxSPI = bk72xx_spi_ns.class_("BK72xxSPI", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BK72xxSPI),
        }
    ),
    libretiny.only_on_family(
        supported=[libretiny.const.FAMILY_BK7238, libretiny.const.FAMILY_BK7231N]
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
