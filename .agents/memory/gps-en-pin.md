---
name: Heltec V4 L76K GPS EN pin
description: The GPS enable pin (GPIO34) on Heltec WiFi LoRa 32 V4 is active HIGH, not active LOW.
---

## Rule
`PIN_GPS_EN` (GPIO34) on the Heltec WiFi LoRa 32 V4 + L76K GPS module is **active HIGH**.

- `powerOn()`:  `digitalWrite(PIN_GPS_EN, HIGH)` — enables the module
- `powerOff()`: `digitalWrite(PIN_GPS_EN, LOW)`  — cuts power

**Why:** The original code had `LOW` to enable with a comment "active low". The module produced zero NMEA output, confirmed by `gpstest`. Flipping to HIGH fixed it. The Meshtastic heltec_v4 variant also drives it HIGH.

**How to apply:** Any future GPS power-control code for the Heltec V4 must use HIGH=on, LOW=off for GPIO34.
