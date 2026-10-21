#include "bk72xx_spi.h"

#include "esphome/core/log.h"

namespace esphome::bk72xx_spi {

static const char *const TAG = "bk72xx_spi";

static constexpr size_t SPI_MAX_LENGTH = 4095;

#ifdef USE_BK72XX

extern "C" {
#include "rtos_pub.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "gpio_pub.h"
#include "icu_pub.h"
}

// BK7231N/BK7238 SPI master. LibreTiny builds the Beken driver with CFG_USE_SPI=0,
// so bk_spi_master_xfer() is not linked. These are the registers from spi_bk7231n.h.
// The included SDK headers may define the older BK7231U map under the same names.
#undef SPI_BASE
#undef SPI_CTRL
#undef SPI_CONFIG
#undef SPI_STAT
#undef SPI_DAT
static constexpr uint32_t SPI_BASE = 0x00802700;
static constexpr uint32_t SPI_CTRL = SPI_BASE + 0 * 4;
static constexpr uint32_t SPI_CONFIG = SPI_BASE + 1 * 4;
static constexpr uint32_t SPI_STAT = SPI_BASE + 2 * 4;
static constexpr uint32_t SPI_DAT = SPI_BASE + 3 * 4;

static constexpr uint32_t SPI_CKR_POS = 8;
static constexpr uint32_t SPI_NSS_3WIRE = 1u << 17;
static constexpr uint32_t SPI_LSB_FIRST = 1u << 19;
static constexpr uint32_t SPI_CKPOL = 1u << 20;
static constexpr uint32_t SPI_CKPHA = 1u << 21;
static constexpr uint32_t SPI_MSTEN = 1u << 22;
static constexpr uint32_t SPI_ENABLE = 1u << 23;
static constexpr uint32_t SPI_BYTE_INTERVAL_POS = 24;

static constexpr uint32_t SPI_TX_ENABLE = 1u << 0;
static constexpr uint32_t SPI_RX_ENABLE = 1u << 1;
static constexpr uint32_t SPI_TX_LEN_POS = 8;
static constexpr uint32_t SPI_RX_LEN_POS = 20;

static constexpr uint32_t SPI_TX_FIFO_WRITABLE = 1u << 1;
static constexpr uint32_t SPI_RX_FIFO_READABLE = 1u << 2;
static constexpr uint32_t SPI_TX_FIFO_CLEAR = 1u << 16;

static constexpr uint32_t SPI_SOURCE_HZ = 26000000;

static void wait_us(uint32_t us) {
  const uint32_t cycles = (arch_get_cpu_freq_hz() / 1000000u) * us;
  const uint32_t start = arch_get_cpu_cycle_count();
  while ((arch_get_cpu_cycle_count() - start) < cycles) {
  }
}

static void disable_transfer() {
  uint32_t config = REG_READ(SPI_CONFIG);
  config &= ~(SPI_TX_ENABLE | SPI_RX_ENABLE);
  REG_WRITE(SPI_CONFIG, config);
}

bool BK72xxSPI::init_controller_() {
  uint32_t value = PCLK_POSI_SPI;
  sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_26M, &value);
  value = PWD_SPI_CLK_BIT;
  sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &value);

  value = GFUNC_MODE_SPI_GPIO_14;
  sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &value);
  value = GFUNC_MODE_SPI_GPIO_16_17;
  sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &value);

  // Leave the shifter idle. Each transfer writes the clock and mode before asserting CS.
  uint32_t control = SPI_MSTEN | SPI_ENABLE | SPI_NSS_3WIRE;
  control |= 255u << SPI_CKR_POS;
  control |= 2u << SPI_BYTE_INTERVAL_POS;
  REG_WRITE(SPI_CTRL, control);
  REG_WRITE(SPI_CONFIG, 0);
  return true;
}

bool BK72xxSPI::apply_settings_(uint32_t frequency_hz, uint8_t mode, bool lsb_first) {
  if (frequency_hz == 0 || mode > 3) {
    return false;
  }
  // Round the divider up so the clock never runs faster than requested.
  const uint32_t divider = (SPI_SOURCE_HZ / 2u + frequency_hz - 1u) / frequency_hz;
  if (divider < 2 || divider > 255) {
    ESP_LOGE(TAG, "SPI clock %" PRIu32 " Hz is outside 51000-6500000 Hz", frequency_hz);
    return false;
  }
  const uint32_t actual = (SPI_SOURCE_HZ / 2u) / divider;
  uint32_t half_us = (1000000u + actual - 1u) / actual / 2u;
  if (half_us < 2) {
    half_us = 2;
  }
  this->settle_us_ = half_us;

  uint32_t control = SPI_MSTEN | SPI_ENABLE | SPI_NSS_3WIRE;
  control |= divider << SPI_CKR_POS;
  control |= 2u << SPI_BYTE_INTERVAL_POS;
  if ((mode & 0x02) != 0) {
    control |= SPI_CKPOL;
  }
  if ((mode & 0x01) != 0) {
    control |= SPI_CKPHA;
  }
  if (lsb_first) {
    control |= SPI_LSB_FIRST;
  }
  REG_WRITE(SPI_CTRL, control);
  return true;
}

bool BK72xxSPI::fifo_transfer_(const uint8_t *tx, uint8_t *rx, size_t length, uint32_t timeout_ms) {
  // TX clear is a write to the status bit. RX is emptied by reading; the clear
  // bit is not what the Beken driver uses and must not be left set.
  uint32_t status = REG_READ(SPI_STAT);
  REG_WRITE(SPI_STAT, status | SPI_TX_FIFO_CLEAR);
  while ((REG_READ(SPI_STAT) & SPI_RX_FIFO_READABLE) != 0) {
    (void) REG_READ(SPI_DAT);
  }

  size_t queued = 0;
  while (queued < length && (REG_READ(SPI_STAT) & SPI_TX_FIFO_WRITABLE) != 0) {
    const uint8_t byte = tx != nullptr ? tx[queued] : 0;
    REG_WRITE(SPI_DAT, byte);
    queued++;
  }

  uint32_t config = REG_READ(SPI_CONFIG);
  config &= ~((0xFFFu << SPI_TX_LEN_POS) | (0xFFFu << SPI_RX_LEN_POS) | SPI_TX_ENABLE | SPI_RX_ENABLE);
  config |= (static_cast<uint32_t>(length) << SPI_TX_LEN_POS) | (static_cast<uint32_t>(length) << SPI_RX_LEN_POS);
  REG_WRITE(SPI_CONFIG, config);
  config |= SPI_TX_ENABLE | SPI_RX_ENABLE;
  REG_WRITE(SPI_CONFIG, config);

  size_t received = 0;
  const uint32_t start = millis();
  while (received < length) {
    bool progress = false;
    status = REG_READ(SPI_STAT);
    if ((status & SPI_RX_FIFO_READABLE) != 0) {
      const uint8_t byte = static_cast<uint8_t>(REG_READ(SPI_DAT) & 0xFF);
      if (rx != nullptr) {
        rx[received] = byte;
      }
      received++;
      progress = true;
    }
    if (queued < length && (REG_READ(SPI_STAT) & SPI_TX_FIFO_WRITABLE) != 0) {
      const uint8_t byte = tx != nullptr ? tx[queued] : 0;
      REG_WRITE(SPI_DAT, byte);
      queued++;
      progress = true;
    }
    if (!progress && millis() - start > timeout_ms) {
      disable_transfer();
      return false;
    }
  }

  disable_transfer();
  return true;
}

#else

static void wait_us(uint32_t) {}

bool BK72xxSPI::init_controller_() {
  ESP_LOGE(TAG, "BK72xx SPI requires the BK7238 or BK7231N");
  return false;
}

bool BK72xxSPI::apply_settings_(uint32_t, uint8_t, bool) { return false; }

bool BK72xxSPI::fifo_transfer_(const uint8_t *, uint8_t *, size_t, uint32_t) { return false; }

#endif

void BK72xxSPI::setup() {
  this->ready_ = this->init_controller_();
  if (!this->ready_) {
    this->mark_failed();
  }
}

void BK72xxSPI::dump_config() {
  ESP_LOGCONFIG(TAG, "BK72xx SPI:");
  ESP_LOGCONFIG(TAG, "  SCK: P14  MOSI: P16  MISO: P17");
  if (!this->ready_) {
    ESP_LOGCONFIG(TAG, "  Status: FAILED");
  }
}

bool BK72xxSPI::transfer(GPIOPin *cs, uint32_t frequency_hz, uint8_t mode, const uint8_t *tx, uint8_t *rx,
                         size_t length, bool lsb_first) {
  if (!this->ready_ || cs == nullptr || length == 0 || length > SPI_MAX_LENGTH || mode > 3) {
    return false;
  }
  if (!this->apply_settings_(frequency_hz, mode, lsb_first)) {
    return false;
  }
  // 10 bit times per byte, plus the previous 20 ms floor used by short frames.
  const uint32_t timeout_ms = 20u + static_cast<uint32_t>((static_cast<uint64_t>(length) * 10000ull) / frequency_hz);

  cs->digital_write(false);
  wait_us(this->settle_us_);
  const bool ok = this->fifo_transfer_(tx, rx, length, timeout_ms);
  wait_us(this->settle_us_);
  cs->digital_write(true);
  wait_us(this->settle_us_);
  return ok;
}

}  // namespace esphome::bk72xx_spi
