---
name: Heltec V4 L76K GPS EN pin
description: The GPS EN pin (GPIO34) on this board must be left floating — driving it either way kills UART output.
---

## Rule
Do NOT drive `PIN_GPS_EN` (GPIO34) in `powerOn()` or `powerOff()`. Set it `INPUT` and leave it alone.

- `powerOn()`:  `pinMode(PIN_GPS_EN, INPUT)` — hands off
- `powerOff()`: leave unchanged — standby pin controls quiescent draw

The module is always powered from VCC on this board variant; EN is not connected to a power switch or the connection is through a component that the EN drive disrupts.

**Why:** `gpsdiag` showed EN=float → 1 byte received; EN=LOW → 0 bytes; EN=HIGH → 0 bytes. Driving EN either HIGH or LOW as an OUTPUT kills all UART output from the module. Leaving it floating restores data flow.

**How to apply:** Any future `powerOn`/`powerOff` rewrite must not touch GPIO34. Use the standby pin (GPIO40) for sleep control instead.

## Baud Rate
L76K default is 9600. If `gpsdiag` baud scan shows ~100% printable bytes at a different rate, update `GPS_BAUD` in `GpsUart.cpp`. The `\xC0` byte seen at 9600 during initial testing was likely a mid-byte capture artifact at startup, not a mismatch indicator.
