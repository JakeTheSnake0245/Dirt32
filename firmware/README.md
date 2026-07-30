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
        ├── radio/LoRaLink        SX1262 TX + closed-loop ACK window
        └── display/DebugScreen   PRG-button OLED page: link params + key FP
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

## Configuring a node (serial CLI)

Cold-boot (USB power / reset button) drops the node into bench mode with a
CLI on the serial port. Connect with `pio device monitor -b 115200`, then:

```
show                       # print full config as JSON (PSK redacted)
set <param> <value>        # change one parameter
save                       # persist to flash (survives power cycles)
reboot                     # apply anything the radio reads at init
```

### Radio parameters — MUST match on every board in a network

| Param      | Meaning                              | Default | Example            |
|------------|--------------------------------------|---------|--------------------|
| `f_in`     | TX frequency, MHz (node → uplink)    | 903.0   | `set f_in 903.0`   |
| `sf`       | Spreading factor (7–12)              | 10      | `set sf 9`         |
| `bw`       | Bandwidth, kHz                       | 125.0   | `set bw 125.0`     |
| `cr`       | Coding rate 4/x (5–8)                | 5       | `set cr 5`         |
| `net_id`   | Network ID (drives LoRa sync word)   | 1       | `set net_id 7`     |
| `psk`      | 256-bit key, 64 hex chars            | zeros   | `set psk <hex>` or `keygen` |

Also matching-relevant: `ack_enable` (the receiver must actually source ACKs
for the sender's ACK window to close).

Per-board (must NOT match): `node_id` — unique per node.
`tx_power` (dBm, ≤22) affects range only, not compatibility.

### Heartbeat periodicity

```
set heartbeat_per_day 4    # 4 = every 6 h; 24 = hourly; 96 = every 15 min
save
```

The node sleeps between beats and wakes on a timer; each heartbeat is a
single encrypted, single-shot frame (no retransmit burst). On the bench,
`hb` sends one immediately without waiting for the timer.

### Detection / front-end parameters

`front_end` (`adxl355` | `geophone`), `sample_rate_hz`, `hpf_hz`, `sta_ms`,
`lta_ms`, `trigger_ratio`, `footstep_lo`/`footstep_hi`, `vehicle_lo`/
`vehicle_hi`, `motion_wake_enable`, `motion_threshold`. These only matter on
the sending node — they never affect radio compatibility.

### Reliability parameters (alerts)

`ack_enable`, `ack_window_ms`, `retx_count`, `retx_jitter_min`,
`retx_jitter_max`.

## Debug screen (PRG button)

Press the **PRG (USER) button** and the onboard OLED shows everything that
must match for two boards to communicate:

```
NET 7   NODE 1
FREQ 903.0 MHz
SF9 BW125k CR4/5
KEY 3F0A   ACK ON
HB 4/day  TX 17dBm
SEQ 129  RADIO OK
```

- **KEY** is a 4-hex-digit fingerprint of the PSK — the key itself is never
  shown. Two boards displaying the same fingerprint hold the same key.
- Hold both boards side by side: if lines 1–4 match (NET, FREQ, SF/BW/CR,
  KEY), they can talk. NODE must differ; SEQ/RADIO are per-board status.
- Press PRG again to hide it; it auto-blanks after 20 s. `screen` on the CLI
  shows the same page.
- The OLED is only ever powered in bench mode — it stays dark in deployed
  (sleep/wake) operation.

### Verify before trusting (spec §12)

- **Ve polarity (GPIO36):** default assumed active-HIGH. If your board is
  the opposite, add `-DVE_ACTIVE_LOW` to `build_flags`.
- **Pin map:** LoRa + sensor pins in `platformio.ini` — check against your
  V4 silk/schematic and harness before first power-up.
- **Battery divider:** calibrate `readBatteryMv()` against a meter.

## Security model (spec §4.2, §10)

- Per-node 256-bit PSK; ChaCha20-Poly1305 with the 8-byte header as AAD.
- Nonce = `NODE_ID(2) ‖ MSG_TYPE(1) ‖ SEQ(3) ‖ 0*6` — unique because SEQ is
  monotonic and **reserved in NVS before use** (write-ahead, 64 at a time),
  fast-forwarded to the NVS bound on *every* reset cause, and hard-stopped
  before 24-bit wrap (rotate the PSK, then `seqreset`) — so no brownout,
  watchdog, or counter wrap can ever cause nonce reuse.
- Node authenticates ACKs under its own key and runs its own replay window,
  so a captured/replayed ACK cannot suppress retransmissions.
- Spoofed `NODE_ID`/`SEQ` (header tamper) fails tag verification: header is
  AAD *and* drives the nonce.

## Next tiers (not yet built)

- Relay firmware (6b single-radio first) — reuses `sps_proto` dedupe as-is.
- Gateway ingest (RPi) — reuses `sps_proto` seal/open + replay as-is.
- Portal.
