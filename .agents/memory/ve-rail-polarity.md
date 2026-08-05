---
name: Heltec V4 Ve/Vext rail polarity + parasitic-backfeed failure signature
description: GPIO36 VextCtrl is ACTIVE HIGH per V4.3.1 datasheet; wrong polarity makes SPI sensors run on diode backfeed and brown out at analog-engine turn-on.
---

## Rule
Heltec WiFi LoRa 32 V4: **GPIO36 (VextCtrl) is ACTIVE HIGH** — datasheet V4.3.1 §3.3: "When using VE for external power supply, the VextCtrl (GPIO36) pin needs to be pulled high." Do NOT trust V3-era Meshtastic lore (V3 Vext was active low). Note each V4 power switch has its own polarity: VGNSS_Ctrl (GPIO34) is active LOW — verify each against the datasheet, never by analogy.

**Why:** with polarity inverted, the Ve switch is off and an SPI slave runs on parasitic power back-fed through its bus protection diodes. Digital-only traffic (ID reads, config writes, NVM) works perfectly at µA loads; the chip "browns out" the instant a real load starts (ADXL355 analog engine at measurement entry, ~200 µA + surge) — interface goes fully dead (DEVID=0x00, MISO pinned low, inputs deaf) until the parasitic charge is drained. A DMM on the unloaded rail reads ~3.29 V (floats near full with no load), which falsely "proves" the supply is fine.

**How to apply:** if a bus device passes every digital check but dies deterministically the moment its analog section powers up — at any SPI speed, on multiple boards, reviving only after real power drain — suspect the power-switch polarity / parasitic backfeed FIRST, before chip latch-up, signal integrity, or counterfeit theories. Confirm by measuring the rail *under load* or toggling the switch polarity.
