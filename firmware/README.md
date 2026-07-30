# Dirt32 — Distributed Seismic Perimeter Sensor Firmware

Buried LoRa sensor nodes that detect footsteps and vehicles and report them
over an encrypted, replay-protected radio link.

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

Defaults ARE the recommended settings for this deployment (buried perimeter
nodes, US915, long battery life) — a freshly flashed board only needs
`node_id`, `net_id`, and a key.

| Param    | Meaning                            | Min  | Max   | Default = recommended | Why |
|----------|------------------------------------|------|-------|-----------------------|-----|
| `f_in`   | TX freq MHz (node → uplink)        | 902.3| 914.9 | **903.0**             | Clear of the US915 LoRaWAN uplink center channels |
| `sf`     | Spreading factor                   | 7    | 12    | **10**                | Buried antenna + ~1 km field range; SF7 is bench-only, SF12 burns 4× the battery per frame |
| `bw`     | Bandwidth kHz                      | 62.5 | 500   | **125.0**             | Best sensitivity/airtime balance at SF10 |
| `cr`     | Coding rate 4/x                    | 5    | 8     | **5**                 | Higher CR only helps in heavy interference; costs airtime |
| `net_id` | Network ID → LoRa sync word        | 1    | 255   | **1** (pick your own) | Isolates your perimeter from any neighboring deployment |
| `psk`    | 256-bit key (64 hex)               | —    | —     | `keygen` per node     | Never reuse a key across nodes |

Also matching-relevant: `ack_enable` (the receiver must actually source ACKs
for the sender's ACK window to close).

Per-board (must NOT match): `node_id` — unique per node.

| Param      | Min | Max | Default = recommended | Why |
|------------|-----|-----|-----------------------|-----|
| `tx_power` | 2   | 22  | **20** dBm            | Soil attenuation eats margin; 22 only if batteries allow — doesn't affect compatibility |

### Heartbeat periodicity

| Param               | Min | Max | Default = recommended | Why |
|---------------------|-----|-----|-----------------------|-----|
| `heartbeat_per_day` | 1   | 96  | **24** (hourly)       | A security perimeter should notice a dead node within ~3 h (offline = 3 missed beats); 96 halves ADXL battery life for little gain, 4 leaves you blind most of a day |

```
set heartbeat_per_day 24   # 24 = hourly; 4 = every 6 h; 96 = every 15 min
save
```

The node sleeps between beats and wakes on a timer; each heartbeat is a
single encrypted, single-shot frame (no retransmit burst). On the bench,
`hb` sends one immediately without waiting for the timer.

Heartbeats carry position, battery, tamper/motion-since-last-beat, noise
floor, firmware version, and reset count.

### GPS (L76K GNSS plug-in module)

The firmware drives the Heltec L76K module on the V4's GNSS port directly
(UART GPIO38/39, power-enable GPIO34 active-low, standby GPIO40, reset
GPIO42 — matching the V4 schematic). Behavior:

- At each heartbeat (if `gps_enable 1`), the module is powered, given up to
  `gps_fix_timeout_s` (default 60 s) to produce a fix, then **fully powered
  off** — it draws nothing between beats.
- On a fix, the heartbeat carries real coordinates and the node's clock is
  synced to UTC, so alert timestamps are real time from then on.
- No fix (buried antenna, cold start, no sky view) → falls back to the
  provisioned `fallback_lat_e7`/`fallback_lon_e7`
  (`set fallback_lat_e7 407128000` = 40.7128000°N). For buried nodes,
  setting the fallback and `set gps_enable 0` saves the 60 s fix budget
  every beat — position doesn't change once planted.
- Bench check: `gpstest` streams raw NMEA for 30 s (you should see
  `$GxRMC`/`$GxGGA` sentences immediately, status `A` once locked, outdoors);
  `gpsfix` runs a full parsed acquisition and prints lat/lon/sats/HDOP.

| Param               | Min | Max | Default = recommended | Why |
|---------------------|-----|-----|-----------------------|-----|
| `gps_enable`        | 0   | 1   | **1** (0 once planted)| Useful during install/survey; a buried node's position never changes, so turn it off and rely on the fallback to save battery |
| `gps_fix_timeout_s` | 10  | 300 | **60**                | L76K cold start is 30-60 s with sky view; longer just drains the battery when buried |

### Detection / front-end parameters (sender-only, never affect compatibility)

| Param               | Min  | Max   | Default = recommended | Why |
|---------------------|------|-------|-----------------------|-----|
| `front_end`         | —    | —     | **adxl355**           | Motion-wake deep sleep = months of battery; use `geophone` only where you need max sensitivity and have the power budget |
| `sample_rate_hz`    | 100  | 500   | **250**               | Covers the 80 Hz footstep band with margin; 500 doubles CPU-awake time |
| `hpf_hz`            | 1    | 8     | **2.0**               | Kills wind/thermal drift without touching the 2–20 Hz vehicle band |
| `sta_ms`            | 50   | 1000  | **200**               | Short enough to catch a single footstep impulse |
| `lta_ms`            | 1000 | 30000 | **5000**              | Stable background estimate; longer adapts too slowly at dawn/dusk noise shifts |
| `trigger_ratio`     | 2    | 10    | **4.0**               | Start here, then tune with `detector 30` on-site (spec §13): lower → sensitive, higher → fewer false alarms |
| `footstep_lo`/`hi`  | —    | —     | **20 / 80** Hz        | Per spec §5.2 band split |
| `vehicle_lo`/`hi`   | —    | —     | **2 / 20** Hz         | Per spec §5.2 band split |
| `motion_threshold`  | 0.002| 0.1   | **0.005** g           | ADXL wake threshold; raise if wind/livestock cause spurious wakes |

### Reliability parameters (alerts)

| Param             | Min | Max  | Default = recommended | Why |
|-------------------|-----|------|-----------------------|-----|
| `ack_enable`      | 0   | 1    | **1**                 | Closed-loop delivery for a security system |
| `ack_window_ms`   | 200 | 5000 | **1500**              | SF10/125 kHz ACK airtime is ~400 ms; 500 was too tight, 5000 wastes RX battery |
| `retx_count`      | 1   | 8    | **3**                 | 3 tries at jittered intervals beats one packet through a collision |
| `retx_jitter_min` | 50  | —    | **200** ms            | Randomized backoff so two triggered nodes don't collide repeatedly |
| `retx_jitter_max` | —   | 5000 | **800** ms            | Keeps worst-case alert latency ~4 s at retx 3 |

## Debug screen (PRG button)

Press the **PRG (USER) button** and the onboard OLED shows everything that
must match for two boards to communicate:

```
NET 7   NODE 1
FREQ 903.0 MHz
SF9 BW125k CR4/5
KEY 3F0A   ACK ON
HB 24/day  TX 20dBm
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

## Wiring the seismic sensors

Both drivers are built and selectable at runtime — no reflash needed:
`set front_end adxl355` or `set front_end geophone`, then `save`, `reboot`,
and `selftest` to verify. Both sensors share the SPI bus with the LoRa
radio (SCK = GPIO9, MISO = GPIO11, MOSI = GPIO10); each device gets its own
chip-select.

> ⚠️ GPIO 2, 5, 7, and 46 are used **internally** by the V4's RF power
> amplifier. Never connect anything to them.

### Option A — ADXL355 (digital, SPI) — recommended first

| ADXL355 pin | V4 pin | Notes |
|-------------|--------|-------|
| VDD + VDDIO | Ve (3.3 V switched) | powered down during sleep between events |
| GND         | GND    | |
| SCLK        | GPIO9  | shared SPI clock |
| MOSI (SDA)  | GPIO10 | shared |
| MISO (SDO)  | GPIO11 | shared |
| CS          | GPIO26 | dedicated chip-select |
| INT1        | GPIO6  | motion-wake interrupt (RTC-capable → wakes from deep sleep) |

### Option B — SM-24 geophone + ADS1220 (analog front-end)

| ADS1220 pin | V4 pin | Notes |
|-------------|--------|-------|
| DVDD/AVDD   | Ve (3.3 V switched) | |
| GND         | GND    | |
| SCLK        | GPIO9  | shared SPI clock |
| DIN (MOSI)  | GPIO10 | shared |
| DOUT (MISO) | GPIO11 | shared |
| CS          | GPIO33 | dedicated chip-select |
| DRDY        | GPIO4  | data-ready, polled |
| AIN0 / AIN1 | SM-24 coil ± | differential input, PGA gain 32 |

The SM-24 connects directly to the ADS1220's differential inputs (add the
usual damping resistor across the coil per the SM-24 datasheet, ~1 kΩ).

### Bench verification order

1. Wire the sensor, cold-boot to the CLI.
2. `selftest` — checks device ID over SPI (ADXL355) / conversion sanity.
3. `detector 30` — streams the live STA/LTA ratio; tap the desk and watch
   it spike. Tune `trigger_ratio` until desk-taps trigger and footsteps of
   people walking nearby behave the way you want.
4. `taptest` — full chain: detection event → encrypted ALERT over the air.
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
