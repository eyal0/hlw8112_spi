#include "hlw8112_spi.h"

#include "esphome/core/log.h"

#include <cmath>

namespace esphome::hlw8112_spi {

static const char *const TAG = "hlw8112_spi";

static constexpr uint32_t ENERGY_MAGIC = 0x484C5731;
static constexpr uint8_t SAVE_EVERY_UPDATES = 30;
static constexpr size_t SPI_MAX_TRANSFER = 16;

static void wait_us(uint32_t us) {
  const uint32_t cycles = (arch_get_cpu_freq_hz() / 1000000u) * us;
  const uint32_t start = arch_get_cpu_cycle_count();
  while ((arch_get_cpu_cycle_count() - start) < cycles) {
  }
}

static constexpr uint8_t REG_SYSCON = 0x00;
static constexpr uint8_t REG_EMUCON = 0x01;
static constexpr uint8_t REG_HFCONST = 0x02;
static constexpr uint8_t REG_EMUCON2 = 0x13;
static constexpr uint8_t REG_ANGLE = 0x22;
static constexpr uint8_t REG_UFREQ = 0x23;
static constexpr uint8_t REG_RMSIA = 0x24;
static constexpr uint8_t REG_RMSIB = 0x25;
static constexpr uint8_t REG_RMSU = 0x26;
static constexpr uint8_t REG_POWER_FACTOR = 0x27;
static constexpr uint8_t REG_ENERGY_PA = 0x28;
static constexpr uint8_t REG_ENERGY_PB = 0x29;
static constexpr uint8_t REG_POWER_PA = 0x2C;
static constexpr uint8_t REG_POWER_PB = 0x2D;
static constexpr uint8_t REG_POWER_S = 0x2E;
static constexpr uint8_t REG_COEFF_CHECKSUM = 0x6F;
static constexpr uint8_t REG_RMSIAC = 0x70;
static constexpr uint8_t REG_RMSIBC = 0x71;
static constexpr uint8_t REG_RMSUC = 0x72;
static constexpr uint8_t REG_POWER_PAC = 0x73;
static constexpr uint8_t REG_POWER_PBC = 0x74;
static constexpr uint8_t REG_POWER_SC = 0x75;
static constexpr uint8_t REG_ENERGY_AC = 0x76;
static constexpr uint8_t REG_ENERGY_BC = 0x77;

static constexpr uint8_t CMD_ENABLE_WRITE = 0xE5;
static constexpr uint8_t CMD_DISABLE_WRITE = 0xDC;
static constexpr uint8_t CMD_SELECT_A = 0x5A;
static constexpr uint8_t CMD_SELECT_B = 0xA5;
static constexpr uint8_t CMD_PREFIX = 0xEA;

static constexpr uint16_t SYSCON_METER = 0x0F04;  // all ADCs on, PGA IA/IB = 16, PGA U = 1
static constexpr uint16_t EMUCON_METER = 0x1003;  // comparator off, both energy accumulators on
static constexpr uint16_t EMUCON2_METER = 0x00E5;  // IB current, PF, waveform, zero-cross, 1.25 V ref

static constexpr double RMS_I_FULL_SCALE = 8388608.0;    // 2^23
static constexpr double RMS_U_FULL_SCALE = 4194304.0;    // 2^22
static constexpr double POWER_FULL_SCALE = 2147483648.0;  // 2^31
static constexpr double ENERGY_FULL_SCALE = 536870912.0;  // 2^29
static constexpr double PF_FULL_SCALE = 8388607.0;        // 2^23 - 1

static int32_t sign_extend_24(uint32_t value) {
  value &= 0x00FFFFFF;
  if ((value & 0x00800000) != 0) {
    return static_cast<int32_t>(value | 0xFF000000);
  }
  return static_cast<int32_t>(value);
}

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
static constexpr uint32_t SPI_CKPOL = 1u << 20;
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

static bool fifo_transfer(const uint8_t *tx, uint8_t *rx, size_t length) {
  // TX clear is a write to the status bit. RX is emptied by reading; the clear
  // bit is not what the Beken driver uses and must not be left set.
  uint32_t status = REG_READ(SPI_STAT);
  REG_WRITE(SPI_STAT, status | SPI_TX_FIFO_CLEAR);
  while ((REG_READ(SPI_STAT) & SPI_RX_FIFO_READABLE) != 0) {
    (void) REG_READ(SPI_DAT);
  }

  for (size_t i = 0; i < length; i++) {
    const uint32_t start = millis();
    while ((REG_READ(SPI_STAT) & SPI_TX_FIFO_WRITABLE) == 0) {
      if (millis() - start > 20) {
        return false;
      }
    }
    REG_WRITE(SPI_DAT, tx[i]);
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
    if ((REG_READ(SPI_STAT) & SPI_RX_FIFO_READABLE) != 0) {
      const uint8_t byte = static_cast<uint8_t>(REG_READ(SPI_DAT) & 0xFF);
      if (rx != nullptr) {
        rx[received] = byte;
      }
      received++;
    } else if (millis() - start > 20) {
      config = REG_READ(SPI_CONFIG);
      config &= ~(SPI_TX_ENABLE | SPI_RX_ENABLE);
      REG_WRITE(SPI_CONFIG, config);
      return false;
    }
  }

  config = REG_READ(SPI_CONFIG);
  config &= ~(SPI_TX_ENABLE | SPI_RX_ENABLE);
  REG_WRITE(SPI_CONFIG, config);
  return true;
}

bool HLW8112SPI::init_controller_() {
  uint32_t value = PCLK_POSI_SPI;
  sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_26M, &value);
  value = PWD_SPI_CLK_BIT;
  sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &value);

  // SCK = P14, MOSI = P16, MISO = P17. Chip select stays a GPIO (P21 on T1-U-HL).
  value = GFUNC_MODE_SPI_GPIO_14;
  sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &value);
  value = GFUNC_MODE_SPI_GPIO_16_17;
  sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &value);

  // Round the divider up so the clock never runs faster than requested.
  // 500 kHz lands on divider 26. 900 kHz lands on 866.7 kHz, under MCLK/4.
  uint32_t divider = (SPI_SOURCE_HZ / 2u + this->spi_frequency_ - 1u) / this->spi_frequency_;
  if (divider < 2) {
    divider = 2;
  } else if (divider > 255) {
    divider = 255;
  }
  const uint32_t actual = (SPI_SOURCE_HZ / 2u) / divider;

  // Mode 2: clock idles high, slave samples MOSI on the falling edge. 8-bit MSB first, no hardware CS.
  uint32_t control = SPI_MSTEN | SPI_ENABLE | SPI_CKPOL | SPI_NSS_3WIRE;
  control |= divider << SPI_CKR_POS;
  control |= 2u << SPI_BYTE_INTERVAL_POS;
  REG_WRITE(SPI_CTRL, control);
  REG_WRITE(SPI_CONFIG, 0);

  ESP_LOGCONFIG(TAG, "SPI master on P14/P16/P17 at %" PRIu32 " Hz", actual);
  return true;
}

#else

static bool fifo_transfer(const uint8_t *, uint8_t *, size_t) { return false; }

bool HLW8112SPI::init_controller_() {
  ESP_LOGE(TAG, "HLW8112 SPI requires the BK7238 or BK7231N");
  return false;
}

#endif

bool HLW8112SPI::transact_(const uint8_t *tx, uint8_t *rx, size_t length) {
  if (length == 0 || length > SPI_MAX_TRANSFER || this->cs_pin_ == nullptr) {
    return false;
  }
  this->cs_pin_->digital_write(false);
  wait_us(2);
  const bool ok = fifo_transfer(tx, rx, length);
  wait_us(2);
  this->cs_pin_->digital_write(true);
  wait_us(2);
  return ok;
}

bool HLW8112SPI::command_(uint8_t command) {
  const uint8_t tx[2] = {CMD_PREFIX, command};
  return this->transact_(tx, nullptr, sizeof(tx));
}

bool HLW8112SPI::write_register16_(uint8_t address, uint16_t value) {
  const uint8_t tx[7] = {
      CMD_PREFIX,
      CMD_ENABLE_WRITE,
      static_cast<uint8_t>(address | 0x80),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value),
      CMD_PREFIX,
      CMD_DISABLE_WRITE,
  };
  if (!this->transact_(tx, nullptr, sizeof(tx))) {
    ESP_LOGE(TAG, "Write of register 0x%02X failed", address);
    return false;
  }
  uint32_t readback = 0;
  if (!this->read_register_(address, &readback, 2)) {
    return false;
  }
  if (readback != value) {
    ESP_LOGE(TAG, "Register 0x%02X read back 0x%04X, expected 0x%04X", address, static_cast<unsigned>(readback),
             value);
    return false;
  }
  return true;
}

bool HLW8112SPI::read_register_(uint8_t address, uint32_t *value, uint8_t size) {
  uint8_t tx[5] = {static_cast<uint8_t>(address & 0x7F), 0, 0, 0, 0};
  uint8_t rx[5] = {};
  if (!this->transact_(tx, rx, static_cast<size_t>(size) + 1)) {
    return false;
  }
  uint32_t assembled = 0;
  for (uint8_t i = 0; i < size; i++) {
    assembled = (assembled << 8) | rx[i + 1];
  }
  *value = assembled;
  return true;
}

bool HLW8112SPI::configure_chip_() {
  if (!this->write_register16_(REG_SYSCON, SYSCON_METER) || !this->write_register16_(REG_EMUCON, EMUCON_METER) ||
      !this->write_register16_(REG_EMUCON2, EMUCON2_METER)) {
    return false;
  }
  const uint8_t select = this->channel_b_ ? CMD_SELECT_B : CMD_SELECT_A;
  if (!this->command_(select)) {
    ESP_LOGE(TAG, "Channel select failed");
    return false;
  }
  return true;
}

bool HLW8112SPI::load_coefficients_() {
  uint32_t rms_iac = 0;
  uint32_t rms_ibc = 0;
  uint32_t rms_uc = 0;
  uint32_t power_pac = 0;
  uint32_t power_pbc = 0;
  uint32_t power_sc = 0;
  uint32_t energy_ac = 0;
  uint32_t energy_bc = 0;
  uint32_t hfconst = 0;
  uint32_t checksum = 0;
  if (!this->read_register_(REG_RMSIAC, &rms_iac, 2) || !this->read_register_(REG_RMSIBC, &rms_ibc, 2) ||
      !this->read_register_(REG_RMSUC, &rms_uc, 2) || !this->read_register_(REG_POWER_PAC, &power_pac, 2) ||
      !this->read_register_(REG_POWER_PBC, &power_pbc, 2) || !this->read_register_(REG_POWER_SC, &power_sc, 2) ||
      !this->read_register_(REG_ENERGY_AC, &energy_ac, 2) || !this->read_register_(REG_ENERGY_BC, &energy_bc, 2) ||
      !this->read_register_(REG_HFCONST, &hfconst, 2) || !this->read_register_(REG_COEFF_CHECKSUM, &checksum, 2)) {
    ESP_LOGE(TAG, "Failed to read calibration coefficients");
    return false;
  }
  if (rms_uc == 0 || rms_iac == 0 || rms_ibc == 0) {
    ESP_LOGE(TAG, "Coefficient registers read as zero");
    return false;
  }

  const uint16_t sum = static_cast<uint16_t>(0xFFFF + rms_iac + rms_ibc + rms_uc + power_pac + power_pbc + power_sc +
                                              energy_ac + energy_bc);
  const uint16_t expected = static_cast<uint16_t>(~sum);
  if (expected != static_cast<uint16_t>(checksum)) {
    ESP_LOGW(TAG, "Coefficient checksum 0x%04X, chip reports 0x%04X", expected, static_cast<unsigned>(checksum));
  }

  const double k_voltage = this->voltage_ratio_;
  const double k_current_a = this->current_ratio_a_;
  const double k_current_b = this->current_ratio_b_;
  const double k_apparent = this->channel_b_ ? k_current_b : k_current_a;
  this->volts_per_count_ = static_cast<double>(rms_uc) / (k_voltage * RMS_U_FULL_SCALE * 100.0);
  this->amps_per_count_a_ = static_cast<double>(rms_iac) / (k_current_a * RMS_I_FULL_SCALE * 1000.0);
  this->amps_per_count_b_ = static_cast<double>(rms_ibc) / (k_current_b * RMS_I_FULL_SCALE * 1000.0);
  this->watts_per_count_a_ = static_cast<double>(power_pac) / (k_current_a * k_voltage * POWER_FULL_SCALE);
  this->watts_per_count_b_ = static_cast<double>(power_pbc) / (k_current_b * k_voltage * POWER_FULL_SCALE);
  this->va_per_count_ = static_cast<double>(power_sc) / (k_apparent * k_voltage * POWER_FULL_SCALE);
  this->kwh_per_count_a_ =
      static_cast<double>(energy_ac) * static_cast<double>(hfconst) / (k_current_a * k_voltage * ENERGY_FULL_SCALE * 4096.0);
  this->kwh_per_count_b_ =
      static_cast<double>(energy_bc) * static_cast<double>(hfconst) / (k_current_b * k_voltage * ENERGY_FULL_SCALE * 4096.0);

  ESP_LOGCONFIG(TAG,
                "Coefficients IA/IB/U %u/%u/%u, power A/B/S %u/%u/%u, energy A/B %u/%u, HFConst %u",
                static_cast<unsigned>(rms_iac), static_cast<unsigned>(rms_ibc), static_cast<unsigned>(rms_uc),
                static_cast<unsigned>(power_pac), static_cast<unsigned>(power_pbc), static_cast<unsigned>(power_sc),
                static_cast<unsigned>(energy_ac), static_cast<unsigned>(energy_bc), static_cast<unsigned>(hfconst));
  return true;
}

void HLW8112SPI::setup() {
  if (this->cs_pin_ == nullptr) {
    ESP_LOGE(TAG, "CS pin is not set");
    this->mark_failed();
    return;
  }
  this->cs_pin_->setup();
  this->cs_pin_->digital_write(true);

  this->energy_pref_ = global_preferences->make_preference<EnergyStore>(ENERGY_MAGIC);
  this->energy_pref_ready_ = true;
  EnergyStore stored{};
  if (this->energy_pref_.load(&stored) && stored.magic == ENERGY_MAGIC) {
    this->energy_ = stored;
  } else {
    this->energy_.magic = ENERGY_MAGIC;
  }

  if (!this->init_controller_() || !this->configure_chip_() || !this->load_coefficients_()) {
    this->mark_failed();
    return;
  }
  this->ready_ = true;
  this->publish_(this->energy_a_sensor_, this->energy_.import_a);
  this->publish_(this->energy_b_sensor_, this->energy_.import_b);
  this->publish_(this->energy_export_a_sensor_, this->energy_.export_a);
  this->publish_(this->energy_export_b_sensor_, this->energy_.export_b);
}

void HLW8112SPI::publish_(sensor::Sensor *sensor, float value) {
  if (sensor != nullptr) {
    sensor->publish_state(value);
  }
}

void HLW8112SPI::accumulate_energy_(double *total, sensor::Sensor *sensor, double kwh) {
  if (kwh <= 0.0) {
    return;
  }
  *total += kwh;
  this->energy_dirty_ = true;
  this->publish_(sensor, static_cast<float>(*total));
}

void HLW8112SPI::save_energy_(bool force) {
  if (!this->energy_pref_ready_ || !this->energy_dirty_) {
    return;
  }
  if (!force && this->updates_since_save_ < SAVE_EVERY_UPDATES) {
    return;
  }
  this->energy_.magic = ENERGY_MAGIC;
  if (this->energy_pref_.save(&this->energy_) && global_preferences->sync()) {
    this->energy_dirty_ = false;
    this->updates_since_save_ = 0;
  }
}

void HLW8112SPI::reset_energy_a() {
  this->energy_.import_a = 0;
  this->energy_.export_a = 0;
  this->energy_dirty_ = true;
  this->save_energy_(true);
  this->publish_(this->energy_a_sensor_, 0);
  this->publish_(this->energy_export_a_sensor_, 0);
}

void HLW8112SPI::reset_energy_b() {
  this->energy_.import_b = 0;
  this->energy_.export_b = 0;
  this->energy_dirty_ = true;
  this->save_energy_(true);
  this->publish_(this->energy_b_sensor_, 0);
  this->publish_(this->energy_export_b_sensor_, 0);
}

void HLW8112SPI::update() {
  if (!this->ready_) {
    return;
  }

  uint32_t rms_u = 0;
  uint32_t rms_ia = 0;
  uint32_t rms_ib = 0;
  uint32_t power_a = 0;
  uint32_t power_b = 0;
  uint32_t power_s = 0;
  uint32_t energy_a = 0;
  uint32_t energy_b = 0;
  uint32_t ufreq = 0;
  uint32_t power_factor = 0;

  // Energy registers clear when read, so a later failure must not throw away a
  // count that already left the chip.
  const bool rms_ok = this->read_register_(REG_RMSU, &rms_u, 3) && this->read_register_(REG_RMSIA, &rms_ia, 3) &&
                      this->read_register_(REG_RMSIB, &rms_ib, 3);
  const bool power_ok = this->read_register_(REG_POWER_PA, &power_a, 4) &&
                        this->read_register_(REG_POWER_PB, &power_b, 4) &&
                        this->read_register_(REG_POWER_S, &power_s, 4);
  const bool energy_a_ok = this->read_register_(REG_ENERGY_PA, &energy_a, 3);
  const bool energy_b_ok = this->read_register_(REG_ENERGY_PB, &energy_b, 3);
  const bool freq_ok = this->read_register_(REG_UFREQ, &ufreq, 2);
  const bool pf_ok = this->read_register_(REG_POWER_FACTOR, &power_factor, 3);
  const bool ok = rms_ok && power_ok && energy_a_ok && energy_b_ok && freq_ok && pf_ok;
  if (!ok) {
    this->consecutive_failures_++;
    if (this->consecutive_failures_ == 1 || this->consecutive_failures_ == 10) {
      ESP_LOGW(TAG, "SPI read failed");
    }
  } else {
    this->consecutive_failures_ = 0;
  }

  const auto rms_or_zero = [](uint32_t raw) { return (raw & 0x00800000) != 0 ? 0u : raw; };
  float volts = NAN;
  float amps_a = NAN;
  float amps_b = NAN;
  if (rms_ok) {
    volts = static_cast<float>(rms_or_zero(rms_u) * this->volts_per_count_);
    amps_a = static_cast<float>(rms_or_zero(rms_ia) * this->amps_per_count_a_);
    amps_b = static_cast<float>(rms_or_zero(rms_ib) * this->amps_per_count_b_);
  }
  this->publish_(this->voltage_sensor_, volts);
  this->publish_(this->current_a_sensor_, amps_a);
  this->publish_(this->current_b_sensor_, amps_b);

  float watts_a = NAN;
  float watts_b = NAN;
  float va = NAN;
  if (power_ok) {
    watts_a = static_cast<float>(static_cast<int32_t>(power_a) * this->watts_per_count_a_);
    watts_b = static_cast<float>(static_cast<int32_t>(power_b) * this->watts_per_count_b_);
    va = static_cast<float>(std::fabs(static_cast<double>(static_cast<int32_t>(power_s)) * this->va_per_count_));
  }
  this->publish_(this->power_a_sensor_, watts_a);
  this->publish_(this->power_b_sensor_, watts_b);
  this->publish_(this->apparent_power_sensor_, va);

  float hertz = NAN;
  if (freq_ok) {
    hertz = ufreq == 0 ? 0.0f : static_cast<float>((static_cast<double>(this->chip_clock_) / 8.0) / ufreq);
  }
  this->publish_(this->frequency_sensor_, hertz);
  this->publish_(this->power_factor_sensor_,
                 pf_ok ? static_cast<float>(sign_extend_24(power_factor) / PF_FULL_SCALE) : NAN);

  if (this->phase_angle_sensor_ != nullptr) {
    uint32_t angle = 0;
    if (freq_ok && this->read_register_(REG_ANGLE, &angle, 2)) {
      const float degrees_per_count = hertz >= 55.0f ? 0.0965f : 0.0805f;
      this->publish_(this->phase_angle_sensor_, static_cast<float>(angle) * degrees_per_count);
    } else {
      this->publish_(this->phase_angle_sensor_, NAN);
    }
  }

  if (energy_a_ok) {
    const double kwh_a = static_cast<double>(energy_a) * this->kwh_per_count_a_;
    if (!power_ok || watts_a >= 0.0f) {
      this->accumulate_energy_(&this->energy_.import_a, this->energy_a_sensor_, kwh_a);
    } else {
      this->accumulate_energy_(&this->energy_.export_a, this->energy_export_a_sensor_, kwh_a);
    }
  }
  if (energy_b_ok) {
    const double kwh_b = static_cast<double>(energy_b) * this->kwh_per_count_b_;
    if (!power_ok || watts_b >= 0.0f) {
      this->accumulate_energy_(&this->energy_.import_b, this->energy_b_sensor_, kwh_b);
    } else {
      this->accumulate_energy_(&this->energy_.export_b, this->energy_export_b_sensor_, kwh_b);
    }
  }
  this->updates_since_save_++;
  this->save_energy_(false);

  if (ok) {
    ESP_LOGD(TAG, "V=%.1f V  Ia=%.3f A  Ib=%.3f A  Pa=%.1f W  Pb=%.1f W  f=%.2f Hz", volts, amps_a, amps_b, watts_a,
             watts_b, hertz);
  }
}

void HLW8112SPI::on_shutdown() { this->save_energy_(true); }

void HLW8112SPI::dump_config() {
  ESP_LOGCONFIG(TAG, "HLW8112 SPI:");
  LOG_PIN("  CS Pin: ", this->cs_pin_);
  ESP_LOGCONFIG(TAG, "  SCK: P14  MOSI: P16  MISO: P17");
  ESP_LOGCONFIG(TAG, "  Shared channel: %s", this->channel_b_ ? "B" : "A");
  ESP_LOGCONFIG(TAG, "  Current ratio A/B: %.4f / %.4f", this->current_ratio_a_, this->current_ratio_b_);
  ESP_LOGCONFIG(TAG, "  Voltage ratio: %.4f", this->voltage_ratio_);
  ESP_LOGCONFIG(TAG, "  Chip clock: %" PRIu32 " Hz", this->chip_clock_);
  if (!this->ready_) {
    ESP_LOGCONFIG(TAG, "  Status: FAILED");
  }
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Frequency", this->frequency_sensor_);
  LOG_SENSOR("  ", "Current A", this->current_a_sensor_);
  LOG_SENSOR("  ", "Current B", this->current_b_sensor_);
  LOG_SENSOR("  ", "Power A", this->power_a_sensor_);
  LOG_SENSOR("  ", "Power B", this->power_b_sensor_);
  LOG_SENSOR("  ", "Apparent Power", this->apparent_power_sensor_);
  LOG_SENSOR("  ", "Power Factor", this->power_factor_sensor_);
  LOG_SENSOR("  ", "Phase Angle", this->phase_angle_sensor_);
  LOG_SENSOR("  ", "Energy A", this->energy_a_sensor_);
  LOG_SENSOR("  ", "Energy B", this->energy_b_sensor_);
  LOG_SENSOR("  ", "Energy Export A", this->energy_export_a_sensor_);
  LOG_SENSOR("  ", "Energy Export B", this->energy_export_b_sensor_);
}

}  // namespace esphome::hlw8112_spi
