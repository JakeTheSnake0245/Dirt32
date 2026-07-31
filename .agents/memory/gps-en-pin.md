---
name: Heltec V4 L76K GPS — wiring, EN pin, and UART
description: Physical connector, pin assignments, EN polarity, and gpsdiag pitfalls for the L76K on Heltec WiFi LoRa 32 V4.
---

## Physical connector
The Heltec V4's JTAG header IS the GNSS header — dual-function pins. The L76K module (jumper-wire version) plugs into this header, not the main GPIO header.

## Pin assignments (confirmed correct)
| Signal | GPIO |
|---|---|
| GNSS_RX (ESP32 receives L76K TX) | GPIO38 |
| GNSS_TX (ESP32 sends to L76K RX) | GPIO39 |
| GNSS_EN (power control) | GPIO34 |
| GNSS_WAKEUP / STANDBY | GPIO40 |
| GNSS_RST | GPIO42 |

## EN pin polarity
GPIO34 HIGH = module powered on. LOW = module dead. Confirmed by Heltec schematic and community reports. Do NOT drive EN LOW to power on.

**Why:** Earlier confusion came from a community code snippet using LOW; the official Heltec factory example uses HIGH. gpsdiag EN=HIGH confirmed bytes received vs EN=LOW = zero.

## JST cable orientation
The L76K's JST cable can be inserted 180° backwards. If GPIO38 reads zero bytes with EN=HIGH, flip the cable and retest before changing firmware.

## gpsdiag pitfall — broad GPIO scanning corrupts UART
Scanning all GPIOs with Serial2.begin/end in a loop reconfigures the ESP32-S3 IO matrix and leaves Serial1 in a bad state. After a broad scan, gpstest always returns zero bytes even if the module is healthy.

**Fix:** gpsdiag now tests ONLY GPIO38/39 with Serial1, then reboots to restore clean UART state. Never re-add broad scanning.

## L76K cold-start timing
- EN=HIGH → ~1 s before first NMEA sentence appears
- Hardware reset pulse (GPIO42 LOW 100ms then HIGH) helps on cold start
- 150 s fix timeout for cold-start outdoors
- Must have clear sky view — no NMEA output ≠ no fix; it means the UART path is broken
