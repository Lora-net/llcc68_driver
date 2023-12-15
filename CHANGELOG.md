# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.3.2] - 2023-12-15

### Changed
- `llcc68_set_gfsk_sync_word()` function - Remove memcpy usage

## [2.3.0] - 2023-10-10

### Added
- `llcc68_set_bpsk_mod_params()` function - Set the modulation parameters for BPSK packets
- `llcc68_set_bpsk_pkt_params()` function - Set the packet parameters for BPSK packets

## [2.2.0] - 2023-03-27

### Added

- `llcc68_driver_version_get_version_string()` function - produces a c-string representation of the driver version
- `LLCC68_DRIVER_VERSION_CHECK` macro - validates the provided version information is compatible with the driver

## [2.1.0] - 2022-05-18

### Added

- `llcc68_set_gfsk_pkt_address()` function - configure both GFSK node and brodcast filtering addresses
- `llcc68_handle_rx_done()` function - perform all requested actions when the chip leaves the Rx mode

## [2.0.1] - 2021-11-23

### Added

- `llcc68_get_lora_params_from_header()` function - extracts the LoRa coding rate and CRC configuration from the packet header
- `llcc68_add_registers_to_retention_list()` function - allows to add up to 4 registers to the retention list
- `llcc68_init_retention_list()` - add registers used by workarounds in the driver to the retention list
- `llcc68_cal_img_in_mhz()` - takes frequency interval bounds in MHz for calibration

### Changed

- Revised BSD License changed to the Clear BSD License
- `llcc68_set_lora_symb_nb_timeout` now rounds up to nearest possible number of symbol
- `LLCC68_REG_IRQ_POLARITY` is renamed `LLCC68_REG_IQ_POLARITY`
- `llcc68_cal_img()` function - takes frequency interval bounds in raw steps

## [1.0.0] - 2020-09-24

### General

- Initial version
