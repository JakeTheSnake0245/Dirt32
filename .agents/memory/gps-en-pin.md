---
name: Heltec V4 L76K GPS EN pin polarity
description: GPIO34 (VGNSS_Ctrl) is ACTIVE LOW — LOW powers the GPS on. Earlier "active HIGH" belief was wrong and kept the GPS off.
---

## Rule
GPIO34 (VGNSS_Ctrl) is **ACTIVE LOW**: drive LOW to power the L76K, HIGH to cut power.

**Why:** Meshtastic's field-proven variants define `GPS_EN_ACTIVE LOW` for both heltec_v4 (EN=GPIO34) and heltec_v4_r8 (EN=GPIO42). A known Meshtastic issue documents that firmware assuming the usual active-HIGH polarity turns the Heltec GNSS rail *off*. Our firmware ran EN=HIGH for the entire debugging saga — the GPS was never powered.

**How to apply:** `powerOn()` drives EN LOW; `powerOff()` drives EN HIGH. R8 revision boards move EN to GPIO42 (also active LOW) — `gpsraw` sub-test E detects this, sub-test F detects a genuinely inverted (active-HIGH) third-party module.
