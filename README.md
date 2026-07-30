# Dirt32

**Buried seismic perimeter sensors over encrypted LoRa.**

Dirt32 is a distributed perimeter-security system: low-power sensor nodes
buried at a property line detect footsteps and vehicles through ground
vibration and report them by radio — no wires, no Wi-Fi, months on a
battery.

## How it works

```
[buried node] --LoRa 903 MHz--> [relay] --927 MHz--> [gateway] --> [map/alerts]
 ESP32-S3 + SX1262                                    (planned)      (planned)
 ADXL355 or geophone
```

- **Detection** — STA/LTA trigger on the seismic signal, with a band-split
  classifier (footsteps 20–80 Hz vs vehicles 2–20 Hz) and per-event
  confidence.
- **Security** — every frame is ChaCha20-Poly1305 encrypted and
  authenticated with a per-node 256-bit key. The header is bound as AAD,
  sequence numbers are write-ahead-reserved in flash so no reset or
  brownout can ever reuse a nonce, and receivers enforce a sliding replay
  window. Alerts use closed-loop authenticated ACKs with jittered
  retransmits.
- **Power** — nodes deep-sleep and wake on sensor motion interrupt or a
  heartbeat timer; the L76K GPS module is only powered during a fix.

## Hardware

| Part | Role |
|------|------|
| Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262) | node MCU + radio |
| ADXL355 accelerometer *or* SM-24 geophone + ADS1220 | seismic front-end |
| Heltec L76K GNSS module | position + UTC time (optional) |

## Repository layout

```
firmware/
├── lib/sps_proto/   Shared wire protocol + crypto (pure C99, no deps)
├── node/            Sensor node firmware (PlatformIO)
└── test/host/       Protocol/crypto test suite — runs on any machine, no hardware
```

## Quick start

```sh
# run the protocol + crypto tests (no hardware needed)
cd firmware/test/host
gcc -std=c99 -Wall -Wextra -O2 -I../../lib/sps_proto \
    ../../lib/sps_proto/chacha20poly1305.c ../../lib/sps_proto/sps_proto.c \
    test_main.c -o sps_test && ./sps_test

# flash a node
cd firmware/node && pio run -t upload && pio device monitor -b 115200
```

Then on the serial CLI: `keygen`, `set node_id 1`, `set net_id 7`, `save` —
and press the **PRG button** to get an OLED debug page showing every
parameter (plus a key fingerprint) that must match between boards.

**Full documentation — configuration reference with recommended settings,
bench workflow, GPS, debug screen, security model — is in
[`firmware/README.md`](firmware/README.md).**

## Status

- ✅ Node firmware + shared protocol library (this repo)
- 🔜 Relay firmware (extends range beyond direct radio reach)
- 🔜 Gateway ingest (decrypt, verify, store)
- 🔜 Live map portal
