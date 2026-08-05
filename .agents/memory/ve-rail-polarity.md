---
name: Heltec V4 Ve/Vext rail polarity
description: GPIO36 VextCtrl is ACTIVE LOW (verified empirically); the V4.3.1 datasheet "pull high" line is about external supply INPUT, not the switched output.
---

## Rule
Heltec WiFi LoRa 32 V4: **GPIO36 (VextCtrl) is ACTIVE LOW** — LOW enables the Ve 3.3 V output, HIGH cuts it. Verified empirically 2026-08-05: driving HIGH made the ADXL355 dead from the first ID read; LOW powers it.

**Why:** the V4.3.1 datasheet sentence "When using VE for external power supply, the VextCtrl (GPIO36) pin needs to be pulled high" reads like active-high but refers to feeding the board FROM an external supply on Ve. Flipping polarity based on that prose cost a flash-and-test cycle. Bench behavior on the actual board is the only trustworthy polarity source.

**How to apply:** `vePower(on)` drives GPIO36 LOW for on. If a sensor answers SPI at all, the rail polarity is correct — do not revisit. Also note each V4 power switch has its own polarity (VGNSS_Ctrl GPIO34 is also active LOW); verify per-switch on the bench, not by datasheet prose or analogy.
