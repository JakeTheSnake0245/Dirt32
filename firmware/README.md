# SPS Firmware — Distributed Seismic Perimeter Sensor System

Implements the node tier and the shared wire protocol from the build spec
(`attached_assets/Pasted--Distributed-Seismic-Perimeter-Sensor-System-*.txt`).

## Layout

```
firmware/
├── lib/sps_proto/        Shared protocol + crypto (pure C99, no deps)
│   ├── sps_proto.h/.c    Frame pack/parse, AEAD seal/open, replay, dedupe
│   └── chacha20poly1305  RFC 8439 AEAD implementation
├── test/host/            Host-side test suite (gcc, runs anywhere)
└── node/                 Sensor node firmware (PlatformIO, Heltec V4)
    └── src/
        ├── main.cpp              Boot/state machine, CLI, sleep logic
        ├── config.h/.cpp         NVS JSON config, all §5.3 parameters
        ├── frontend/             Front-end abstraction
        │   ├── FrontEnd.h        Common interface
        │   ├── Adxl355FrontEnd   Option A — SPI accel, motion-wake
        │   └── GeophoneFrontEnd  Option B — SM-24 + ADS1220, continuous
        ├── detector/StaLta       STA/LTA trigger + band classifier
        └── radio/LoRaLink        SX1262 TX + closed-loop ACK window
```

## Run the protocol tests (host, no hardware)

```sh
cd firmware/test/host
gcc -std=c99 -Wall -Wextra -O2 -I../../lib/sps_proto \
    ../../lib/sps_proto/chacha20poly1305.c ../../lib/sps_proto/sps_proto.c \
    test_main.c -o sps_test && ./sps_test
```

Covers: RFC 8439 AEAD test vectors, little-endian wire layout, frame
seal/open round trips, wrong-key & spoofed-header rejection, replay window
(incl. blind-retransmit dedup), relay dedupe ring, and a full
ALERT→ACK→forged-ACK-rejected scenario.

## Build & flash the node

```sh
cd firmware/node
pio run -t upload        # then: pio device monitor -b 115200
```

### First bench session (two bare V4 boards, no sensors)

1. Flash both boards.
2. On each, over the serial CLI:
   ```
   keygen                     # generates PSK — record the hex, per-node
   set node_id 1              # 2 on the second board
   set net_id 7
   save
   ```
3. On board 1: `taptest` — sends a real encrypted ALERT on f_in (903.0 MHz)
   with retransmit + ACK window. Until a relay/gateway exists it will report
   `sent (no ack)` — that confirms the encrypt→TX path.
4. `detector 30` streams the live STA/LTA ratio once a front-end is wired —
   tap the desk to tune `trigger_ratio` (spec §13 step 4).

### Verify before trusting (spec §12)

- **Ve polarity (GPIO36):** default assumed active-HIGH. If your board is
  the opposite, add `-DVE_ACTIVE_LOW` to `build_flags`.
- **Pin map:** LoRa + sensor pins in `platformio.ini` — check against your
  V4 silk/schematic and harness before first power-up.
- **Battery divider:** calibrate `readBatteryMv()` against a meter.

## Security model (spec §4.2, §10)

- Per-node 256-bit PSK; ChaCha20-Poly1305 with the 8-byte header as AAD.
- Nonce = `NODE_ID(2) ‖ MSG_TYPE(1) ‖ SEQ(3) ‖ 0*6` — unique because SEQ is
  monotonic and **checkpointed to NVS every 64 frames**, fast-forwarded on
  cold boot so a brownout can never cause nonce reuse.
- Node authenticates ACKs under its own key and runs its own replay window,
  so a captured/replayed ACK cannot suppress retransmissions.
- Spoofed `NODE_ID`/`SEQ` (header tamper) fails tag verification: header is
  AAD *and* drives the nonce.

## Next tiers (not yet built)

- Relay firmware (6b single-radio first) — reuses `sps_proto` dedupe as-is.
- Gateway ingest (RPi) — reuses `sps_proto` seal/open + replay as-is.
- Portal.
