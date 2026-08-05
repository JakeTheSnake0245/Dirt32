---
name: CSI ping frame size gate
description: Bench-measured ESP-NOW v2 frame sizes on ESP32-S3 (IDF 5.x) and the CSI length-gate lesson.
---

Rule: never size the CSI frame-length gate from framing math — bench-measure `sig_len` with reject counters first.

**Why:** On IDF 5.x, ESP-NOW v2 vendor action frames with an 8-byte payload arrive at `sig_len = 123 B`, far above the ~50 B that v1 overhead math suggests. Gates of 64 and then 100 B silently rejected the entire peer ping stream (96%+ of callbacks), starving the detector to ~2 fps, causing endless "calibrating" and a near-zero baseline. Beacons bench-measured 150-340+ B.

**How to apply:** Current gate accepts ≤135 B (pings in, beacons out). If detection starves again, use the `csi` CLI's `rx diag` + `sizes` lines (callbacks/accepted/rejHT/rejBig, min/last sizes) to attribute before changing anything. Also: a near-zero calibrated baseline must floor (hyper-sensitive), never mute the metric.
