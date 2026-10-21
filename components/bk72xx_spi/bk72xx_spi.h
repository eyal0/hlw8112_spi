#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::bk72xx_spi {

// One BK7238/BK7231N SPI controller. SCK is P14, MOSI is P16, and MISO is P17.
// Each device supplies its own GPIO chip-select. The clock is 13 MHz divided by
// 2..255, about 51 kHz to 6.5 MHz, and is never faster than requested.
class BK72xxSPI : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  bool is_ready() const { return this->ready_; }

  // Full-duplex transfer. mode is SPI mode 0-3. A null tx buffer clocks out zeros,
  // and a null rx buffer discards what is received. Length is 1..4095 bytes.
  bool transfer(GPIOPin *cs, uint32_t frequency_hz, uint8_t mode, const uint8_t *tx, uint8_t *rx, size_t length,
                bool lsb_first = false);

 protected:
  bool ready_{false};
  uint32_t settle_us_{2};

  bool init_controller_();
  bool apply_settings_(uint32_t frequency_hz, uint8_t mode, bool lsb_first);
  bool fifo_transfer_(const uint8_t *tx, uint8_t *rx, size_t length, uint32_t timeout_ms);
};

}  // namespace esphome::bk72xx_spi
