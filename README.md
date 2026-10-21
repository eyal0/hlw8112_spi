# HLW8112 SPI for ESPHome

ESPHome component for an HLW8112 energy meter wired to the SPI controller of a Beken BK7238 or BK7231N. `t1-u-hl.yaml` is a working config for the Tuya T1-U-HL module.

ESPHome's `spi:` component does not support LibreTiny, and the Beken SDK leaves its SPI driver unlinked. `bk72xx_spi` programs the BK7238/BK7231N SPI controller directly. `hlw8112_spi` is the meter on that bus. It does not bit-bang the bus.

Some of these meter boards tie digital ground to mains. Treat every pin as live.

## Wiring

The SPI controller owns the clock and data pins. Chip select is a normal GPIO because the HLW8112 SCSN pin is not wired to the controller's hardware chip-select.

| Signal | T1-U-HL | Notes |
| --- | --- | --- |
| SCK | P14 | Module pin 1. Fixed by the component. |
| MOSI | P16 | Module pin 2. Fixed by the component. |
| MISO | P17 | Module pin 20. Fixed by the component. |
| SCSN | P21 | Module pin 19. Set with `cs_pin`. GPIO9 on the older CBU. |

SPI mode is mode 2: clock idle high, MSB first. The default clock is 500 kHz.

## Build

Install ESPHome, then create `secrets.yaml` next to the config. It is gitignored.

```yaml
wifi_ssid: "your-network"
wifi_password: "your-password"
```

```bash
pipx install esphome
esphome compile t1-u-hl.yaml
```

The file to flash is the app image, not `firmware.bin` (that file is a UF2 package):

```text
.esphome/build/hlw8112-meter/.pioenvs/hlw8112-meter/image_bk7238_app.0x011000.rbl
```

## Flash

`bk7231tools` writes that image at `0x11000`. Do not write below that address; it is the bootloader.

```bash
bk7231tools write_flash -d /dev/ttyUSB0 -s 0x11000 \
  .esphome/build/hlw8112-meter/.pioenvs/hlw8112-meter/image_bk7238_app.0x011000.rbl
```

Use the serial device from `ls /dev/ttyUSB* /dev/ttyACM*`. The link baud rate is 115200. Start the command, then pulse module pin 18 (CEN) low or power-cycle the module while the tool is trying to connect.

| USB adapter | Module pin | Chip pin |
| --- | --- | --- |
| TX | 16 | P10, UART1 RX |
| RX | 15 | P11, UART1 TX |
| GND | 13 | GND |

Use 3.3 V logic. After boot, the log UART is UART2 (pin 7 is P0 TX, pin 6 is P1 RX). To print logs on the flashing UART instead, set `framework.uart_port: 1` under `bk72xx:`.

## Logs

Once the device is on the network:

```bash
esphome logs t1-u-hl.yaml
```

If `hlw8112-meter.local` does not resolve, pass the IP address with `--device`.

Later firmware updates can use ESPHome OTA. The first install has to go through UART.

## Configuration

Other configs can load the component from GitHub. ESPHome clones it at compile time; there is no separate library to build or copy.

```yaml
# T1-U-HL (BK7238) with the HLW8112 on the chip's SPI pins.
# SCK P14, MOSI P16, and MISO P17 are owned by the SPI controller.
# SCSN is module pin 19, which is P21 on this module (GPIO9 on the older CBU).
#
# Some of these meter boards tie digital ground to mains. Treat every pin as live.

esphome:
  name: hlw8112-meter

bk72xx:
  board: t1-u

external_components:
  - source: github://eyal0/hlw8112_spi@main
    components: [bk72xx_spi, hlw8112_spi]

logger:

api:

ota:
  - platform: esphome

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

bk72xx_spi:
  id: spi_bus

sensor:
  - platform: hlw8112_spi
    id: meter
    spi_id: spi_bus
    cs_pin: P21
    update_interval: 2s
    # K1 / K2 from the HLW8112 calibration-free formulas. These match the
    # clamp-meter ratios used by the OpenBeken driver. Raise a current ratio
    # if that channel reads high.  These are the default values so you can omit them if you want.
    current_ratio_a: 0.2
    current_ratio_b: 0.2
    voltage_ratio: 1.0
    meter_channel: A

    voltage:
      name: Voltage
    frequency:
      name: Frequency
    current_a:
      name: Current A
    current_b:
      name: Current B
    power_a:
      name: Power A
    power_b:
      name: Power B
    # Volt-amps for meter_channel only. The chip has one apparent-power register,
    # so this follows channel A until meter_channel is set to B. Power factor uses
    # that same channel.
    apparent_power:
      name: Apparent Power
    power_factor:
      name: Power Factor
    energy_a:
      name: Energy A
    energy_b:
      name: Energy B

button:
  - platform: template
    name: Reset energy A
    on_press:
      then:
        - lambda: id(meter).reset_energy_a();
  - platform: template
    name: Reset energy B
    on_press:
      then:
        - lambda: id(meter).reset_energy_b();


```

`t1-u-hl.yaml` in this repo keeps `source: type: local` so a compile here uses the files in the checkout. ESPHome caches the GitHub clone for a day; set `refresh: 0s` on that `external_components` entry to pick up a push immediately.

SCK, MOSI, and MISO belong to the one `bk72xx_spi` bus. A second HLW8112 is another `sensor` entry with its own `cs_pin`. Another kind of chip uses that same `spi_id` and its own `cs_pin`. Its driver calls `transfer()` with the clock and SPI mode that chip requires.

| Key | Default | Meaning |
| --- | --- | --- |
| `spi_id` | the only `bk72xx_spi` | Bus this meter uses. |
| `cs_pin` | required | GPIO connected to SCSN. |
| `update_interval` | `10s` | How often registers are read. The example uses `2s`. |
| `spi_frequency` | `500000` | SPI clock, from 100 kHz to 900 kHz. The actual clock is rounded down so it never runs faster than requested. |
| `chip_clock` | `3579545` | HLW8112 crystal, in Hz. Used only to convert the frequency register. |
| `current_ratio_a`, `current_ratio_b` | `0.2` | K1 in the datasheet's calibration-free formulas. These match the OpenBeken clamp-meter defaults. If a current reads high, raise that channel's ratio by the same factor. |
| `voltage_ratio` | `1.0` | K2, the voltage-divider ratio relative to 1 kΩ / 1 MΩ. |
| `meter_channel` | `A` | Which current channel supplies the single apparent-power, power-factor, and phase-angle measurement. |

`current_a`, `current_b`, `power_a`, and `power_b` are independent. Apparent power is volt-amps for `meter_channel` only. The chip has one apparent-power register, so channel B's apparent power is published only when `meter_channel: B`. Power factor uses that same channel.

Optional sensors not included in the example:

- `phase_angle` — degrees, 50 Hz or 60 Hz scale chosen from the measured frequency
- `energy_export_a`, `energy_export_b` — exported kWh

The energy registers clear when they are read, so the component adds each reading into import or export totals and stores them in flash about every 30 updates, and again on shutdown. The template buttons in the example call `id(meter).reset_energy_a()` and `id(meter).reset_energy_b()`.

Supported boards are BK7238 and BK7231N. BK7231T uses a different SPI register map and will be rejected.
