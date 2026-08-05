---
name: Clamped MISO vs dead chip
description: Diagnosing "all SPI reads return 0x00 after mode change" — clamp vs latch vs brownout; PMDZ 2x6 header breadboard trap.
---
The rule: when a SPI device "dies" (all reads 0x00) after a mode change but a **blind write revives it**, the chip is alive — an output pin that only drives in the new mode is clamping MISO through a short. Postmortem toolkit that cracked it: (1) blind standby write, (2) MISO pull-test with internal pullup/pulldown while CS high (pinned low = something driving), (3) re-enter mode with variant configs to fingerprint which pin.

**Why:** Aug 2026 — ADXL355 PMDZ "died" at every measurement entry for multiple sessions. Root cause: PMDZ pin 3 (MISO) bridged to pin 9 (INT2) by the breadboard — 2×6 Pmod headers short vertical pin pairs (1/7, 2/8, 3/9, 4/10, 5/11, 6/12) when both rows sit in one breadboard row group. INT2 idles low only in measurement mode → MISO clamped only then. Chip was genuine and healthy.

**How to apply:** Never seat a 2×6/Pmod header flat on a breadboard row group — span the center channel or dangle on jumpers. And when a user reports running a "modified setup" test, confirm the physical change actually happened (here the "dangling" test was never actually removed from the breadboard, which cost several rounds).
