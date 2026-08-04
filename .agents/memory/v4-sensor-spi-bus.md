---
name: Heltec V4 sensor SPI bus
description: On V4, GPIO9/10/11 (LoRa SPI) have no header pads — sensors need a dedicated SPI bus on exposed pins.
---

## Rule
On Heltec WiFi LoRa 32 V4, the LoRa SPI nets (SCK=9/MOSI=10/MISO=11) are routed internally to the SX1262 and appear on NO header (V4.3 datasheet §2.2 J2/J3/aux tables). External sensors can never share the radio bus — that layout was a V3 carry-over.

**Canonical sensor bus (dedicated HSPI):** SCK=48 (J2 pos 14), MISO=4 (J3 pos 15), MOSI=33 (J2 pos 12). ADXL355: CS=47 (J2 pos 13), INT1=6 (J3 pos 17). ADS1220: CS=46 (J3 pos 5), DRDY=3 (J3 pos 14). GPIO46/3 are strapping pins — no strong external pulls. GPIO33-37 are safe on the V4's ESP32-S3R2 (quad PSRAM); only octal R8 parts claim them. "Jx pos N" is physical header position, NOT the silk GPIO number — always match by silk GPIO.

**Why:** original firmware put sensors on 9/10/11 with separate CS; selftest could never see the sensor no matter the wiring, since those nets have no solderable pad on V4.

**How to apply:** radio keeps default SPI with explicit `SPI.begin(9,11,10,-1)` (GPIO38 clamp rule still holds); sensors use a separate `SPIClass(HSPI)` with explicit pins. Off-limits for sensors: 2/7 (FEM_EN/VFEM_Control), 26–32, 35/37.
