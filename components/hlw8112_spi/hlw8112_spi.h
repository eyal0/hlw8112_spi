#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/bk72xx_spi/bk72xx_spi.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::hlw8112_spi {

// Persisted import/export totals. The HLW8112 energy registers clear on read.
struct EnergyStore {
  uint32_t magic;
  double import_a;
  double export_a;
  double import_b;
  double export_b;
};

class HLW8112SPI : public PollingComponent, public Parented<bk72xx_spi::BK72xxSPI> {
 public:
  void set_cs_pin(GPIOPin *pin) { this->cs_pin_ = pin; }
  void set_spi_frequency(uint32_t frequency) { this->spi_frequency_ = frequency; }
  void set_chip_clock(uint32_t clock) { this->chip_clock_ = clock; }
  void set_current_ratio_a(float ratio) { this->current_ratio_a_ = ratio; }
  void set_current_ratio_b(float ratio) { this->current_ratio_b_ = ratio; }
  void set_voltage_ratio(float ratio) { this->voltage_ratio_ = ratio; }
  void set_channel_b(bool channel_b) { this->channel_b_ = channel_b; }

  void set_voltage_sensor(sensor::Sensor *sensor) { this->voltage_sensor_ = sensor; }
  void set_frequency_sensor(sensor::Sensor *sensor) { this->frequency_sensor_ = sensor; }
  void set_current_a_sensor(sensor::Sensor *sensor) { this->current_a_sensor_ = sensor; }
  void set_current_b_sensor(sensor::Sensor *sensor) { this->current_b_sensor_ = sensor; }
  void set_power_a_sensor(sensor::Sensor *sensor) { this->power_a_sensor_ = sensor; }
  void set_power_b_sensor(sensor::Sensor *sensor) { this->power_b_sensor_ = sensor; }
  void set_apparent_power_sensor(sensor::Sensor *sensor) { this->apparent_power_sensor_ = sensor; }
  void set_power_factor_sensor(sensor::Sensor *sensor) { this->power_factor_sensor_ = sensor; }
  void set_phase_angle_sensor(sensor::Sensor *sensor) { this->phase_angle_sensor_ = sensor; }
  void set_energy_a_sensor(sensor::Sensor *sensor) { this->energy_a_sensor_ = sensor; }
  void set_energy_b_sensor(sensor::Sensor *sensor) { this->energy_b_sensor_ = sensor; }
  void set_energy_export_a_sensor(sensor::Sensor *sensor) { this->energy_export_a_sensor_ = sensor; }
  void set_energy_export_b_sensor(sensor::Sensor *sensor) { this->energy_export_b_sensor_ = sensor; }

  void setup() override;
  void update() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void reset_energy_a();
  void reset_energy_b();

  // False is channel A. Sends the chip's channel-select command once the meter is ready.
  bool set_meter_channel(bool channel_b);
  bool is_channel_b() const { return this->channel_b_; }
  void set_channel_select(select::Select *channel_select) { this->channel_select_ = channel_select; }

 protected:
  GPIOPin *cs_pin_{nullptr};
  uint32_t spi_frequency_{500000};
  uint32_t chip_clock_{3579545};
  float current_ratio_a_{0.2f};
  float current_ratio_b_{0.2f};
  float voltage_ratio_{1.0f};
  bool channel_b_{false};
  bool ready_{false};

  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *current_a_sensor_{nullptr};
  sensor::Sensor *current_b_sensor_{nullptr};
  sensor::Sensor *power_a_sensor_{nullptr};
  sensor::Sensor *power_b_sensor_{nullptr};
  sensor::Sensor *apparent_power_sensor_{nullptr};
  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *phase_angle_sensor_{nullptr};
  sensor::Sensor *energy_a_sensor_{nullptr};
  sensor::Sensor *energy_b_sensor_{nullptr};
  sensor::Sensor *energy_export_a_sensor_{nullptr};
  sensor::Sensor *energy_export_b_sensor_{nullptr};
  select::Select *channel_select_{nullptr};

  // Counts converted with the datasheet calibration-free formulas.
  double volts_per_count_{0};
  double amps_per_count_a_{0};
  double amps_per_count_b_{0};
  double watts_per_count_a_{0};
  double watts_per_count_b_{0};
  double va_per_count_{0};
  double kwh_per_count_a_{0};
  double kwh_per_count_b_{0};
  uint32_t power_sc_{0};

  EnergyStore energy_{};
  ESPPreferenceObject energy_pref_{};
  bool energy_pref_ready_{false};
  bool energy_dirty_{false};
  uint8_t updates_since_save_{0};
  uint8_t consecutive_failures_{0};

  bool configure_chip_();
  bool load_coefficients_();
  void apply_apparent_scale_();
  bool read_register_(uint8_t address, uint32_t *value, uint8_t size);
  bool write_register16_(uint8_t address, uint16_t value);
  bool command_(uint8_t command);
  bool transact_(const uint8_t *tx, uint8_t *rx, size_t length);
  void publish_(sensor::Sensor *sensor, float value);
  void accumulate_energy_(double *total, sensor::Sensor *sensor, double kwh);
  void save_energy_(bool force);
};

// Home Assistant control for the shared channel. Setup runs after the meter.
class MeterChannelSelect : public Component, public select::Select, public Parented<HLW8112SPI> {
 public:
  void setup() override;
  void control(size_t index) override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  ESPPreferenceObject pref_{};
};

}  // namespace esphome::hlw8112_spi
