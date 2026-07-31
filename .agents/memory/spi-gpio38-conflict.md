---
name: SPI clamping GPIO38 (GNSS_RX)
description: SPI.begin() without explicit pins drives GPIO38 LOW via FSPIWP, permanently blocking UART RX on that pin.
---

## Rule
Always call `SPI.begin(/*SCK*/9, /*MISO*/11, /*MOSI*/10, /*SS*/-1)` with explicit pins.

**Why:** On the Heltec V4 (ESP32-S3), `SPI.begin()` without arguments claims GPIO38 as FSPIWP (QSPI write-protect) and drives it LOW. GPIO38 is also `GNSS_RX` for the L76K GPS module. With GPIO38 held LOW by SPI, Serial1.begin() on that pin receives nothing — UART RX is permanently deaf. This was confirmed by `gpio_get_level(38)` returning LOW before `SPI.end()` and HIGH after.

**How to apply:** Any SPI initialization in this project (setup(), any library that calls SPI.begin() internally) must use explicit pin arguments. Verify with `gpio_get_level(38)` if GPS ever stops working again.

## Context
- Board: `heltec_wifi_lora_32_V3` (used as stand-in for V4.3 — no PlatformIO V4 board definition exists)
- No `heltec_wifi_lora_32_V4` board JSON in platformio/platform-espressif32 or HelTecAutomation/Heltec_ESP32
- SPI MISO=11 per V3 variant pins_arduino.h — the MISO assignment is correct; the issue is that SPI.begin() ALSO touches GPIO38 via the QSPI controller regardless of MISO assignment
- Confirmed by: gpsraw sub-test A (GPIO38=LOW, 0 bytes) → SPI.end() → sub-test B (GPIO38=HIGH) → sub-test C (Serial2 got noise byte, confirming module is alive)
