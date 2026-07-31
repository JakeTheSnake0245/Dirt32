---
name: Heltec V4.3 GPS — manufacturer-exact method only
description: GPS UART debugging history; final rule is Serial1 + documented pins only, and how to spot a stale-flash false negative.
---

# Heltec V4.3 (HTIT-WB32LAF) GNSS: use the manufacturer-documented method only

**Rule:** GPS init is exactly Heltec's documented V4.3 method: Serial1, RX=GPIO38, TX=GPIO39, 9600 baud, EN=GPIO34 HIGH, RESET=GPIO42 held HIGH (no pulse), no standby pin. Do not add reset pulses, standby-pin handling, or alternate UART peripherals.

**Why:** A bench loop chased a false "UART1 broken, use Serial2" theory off 1 stray byte (0xC0) — a pin-matrix glitch, not NMEA (9600-baud NMEA is ~400+ bytes/s; require sustained >100 bytes before concluding anything). Community/manufacturer consensus is Serial1 on 38/39 works. The user explicitly demanded the manufacturer method after the experimental detour.

**Watch out — stale flash:** One "test result" was actually the previous firmware build (old log strings, new sub-tests absent). Before trusting bench output, confirm the pasted log contains strings that only exist in the new build.

**Open question:** V4.3 R8 revision moves GNSS enable to GPIO42; user's board is claimed plain V4.3. If GPS stays silent with the manufacturer method on a verified fresh flash, check board revision and TX/RX orientation next.
