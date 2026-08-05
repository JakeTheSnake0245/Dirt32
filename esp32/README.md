# Dirt32 — Distributed Seismic Perimeter Sensor Firmware

Buried LoRa sensor nodes that detect footsteps and vehicles and report them
over an encrypted, replay-protected radio link.

Implements the node tier and the shared wire protocol from the build spec
(`attached_assets/Pasted--Distributed-Seismic-Perimeter-Sensor-System-*.txt`).

## Layout

```
esp32/
├── lib/sps_proto/        Shared protocol + crypto (pure C99, no deps)
│   ├── sps_proto.h/.c    Frame pack/parse, AEAD seal/open, replay, dedupe
│   └── chacha20poly1305  RFC 8439 AEAD implementation
├── test/host/            Host-side test suite (gcc, runs anywhere)
│                         + make_vectors.c — frames for the Python gateway tests
├── gateway-bridge/       USB radio bridge for the Pi gateway (holds no keys)
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
cd esp32/test/host
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
cd esp32/node
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

The firmware drives the Heltec L76K module on the V4's GNSS port directly,
matching Heltec's official V4 GPS example: UART RX = GPIO39, TX = GPIO38
(the silk names are from the module's perspective), power-enable GPIO34
**active-low**, reset GPIO42 held high. Behavior:

- At each heartbeat (if `gps_enable 1`), the module is powered, given up to
  `gps_fix_timeout_s` (default 150 s) to produce a fix, then **fully powered
  off** — it draws nothing between beats.
- On a fix, the heartbeat carries real coordinates and the node's clock is
  synced to UTC, so alert timestamps are real time from then on.
- No fix (buried antenna, cold start, no sky view) → falls back to the
  provisioned fallback position, set in plain decimal degrees:
  `set fallback_lat 40.7128` / `set fallback_lon -74.0060`
  (verify with `show` — it prints them back in degrees). For buried nodes,
  setting the fallback and `set gps_enable 0` saves the 60 s fix budget
  every beat — position doesn't change once planted.
- Bench check: `gpstest` streams raw NMEA for 30 s (you should see
  `$GxRMC`/`$GxGGA` sentences immediately, status `A` once locked, outdoors);
  `gpsfix` runs a full parsed acquisition and prints lat/lon/sats/HDOP.

| Param               | Min | Max | Default = recommended | Why |
|---------------------|-----|-----|-----------------------|-----|
| `gps_enable`        | 0   | 1   | **1** (0 once planted)| Useful during install/survey; a buried node's position never changes, so turn it off and rely on the fallback to save battery |
| `solar_sense_gpio`  | -1  | 5   | **-1** (off)          | Optional. Charging is pure hardware — the V4's charge IC charges the battery whenever 5 V is present (USB or solar), nothing to enable. Set this to a free header GPIO (avoid 2/7 — RF front-end controls — and the sensor-bus pins 48/33/4/47/6/46/3) wired to the panel rail through a ~100k/100k divider, and heartbeats will report the ON_SOLAR flag so the gateway can see the panel producing |
| `gps_fix_timeout_s` | 10  | 300 | **150**                | L76K cold start is 30-60 s with sky view; longer just drains the battery when buried |

### Detection / front-end parameters (sender-only, never affect compatibility)

| Param               | Min  | Max   | Default = recommended | Why |
|---------------------|------|-------|-----------------------|-----|
| `front_end`         | —    | —     | **adxl355**           | Motion-wake deep sleep = months of battery; use `geophone` only where you need max sensitivity and have the power budget |
| `sample_rate_hz`    | 100  | 500   | **250**               | Covers the 80 Hz footstep band with margin; 500 doubles CPU-awake time |
| `hpf_hz`            | 1    | 8     | **2.0**               | Kills wind/thermal drift without touching the 2–20 Hz vehicle band |
| `sta_ms`            | 50   | 1000  | **200**               | Short enough to catch a single footstep impulse |
| `lta_ms`            | 1000 | 30000 | **5000**              | Stable background estimate; longer adapts too slowly at dawn/dusk noise shifts |
| `trigger_ratio`     | 1.5  | 10    | **2.0**               | High-sensitivity default; tune with `detector 30` on-site (spec §13): lower → sensitive, higher → fewer false alarms |
| `footstep_lo`/`hi`  | —    | —     | **20 / 80** Hz        | Per spec §5.2 band split |
| `vehicle_lo`/`hi`   | —    | —     | **2 / 20** Hz         | Per spec §5.2 band split |
| `motion_threshold`  | 0.001| 0.1   | **0.0025** g          | ADXL wake threshold (high-sensitivity default); raise if wind/livestock cause spurious wakes |
| `auto_arm_s`        | 0    | 3600  | **120**               | Cold boot auto-arms after this many untouched-CLI seconds; any keystroke/PRG cancels; 0 = never auto-arm |

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

## Pinout (Heltec WiFi LoRa 32 V4.3, ESP32-S3)

### Board wiring diagram

```
                       Heltec WiFi LoRa 32 V4.3 (ESP32-S3)
                      ┌─────────────────────────────────┐
                      │        [OLED]      [SX1262]     │ SX1262 is on-board:
                      │                    NSS   GPIO8  │ nothing to wire for
   PRG/USER button ───┤ GPIO0              DIO1  GPIO14 │ the radio.
   (debug screen)     │                    RST   GPIO12 │
                      │                    BUSY  GPIO13 │
   VBAT divider ──────┤ GPIO1                           │
                      │                                 │
                       │  SENSOR SPI BUS      GNSS PORT  │      ┌───────────┐
         ┌── SCK ──────┤ GPIO48     GPIO39 (RX ◄─ L76K TX)──────┤  L76K     │
         │   (J2.14)   │            GPIO38 (TX ─► L76K RX)──────┤  GNSS     │
         ├── MOSI ─────┤ GPIO33     GPIO34 (EN, ACTIVE LOW)─────┤  module   │
         │   (J2.12)   │            GPIO42 (RESET, keep HIGH)───┤ (plug-in) │
         ├── MISO ─────┤ GPIO4                           │      └───────────┘
         │   (J3.15)   │                                 │
         │  ┌── CS ────┤ GPIO47   GPIO36 ── Ve rail ──┐  │
         │  │  (J2.13) │          (switched 3.3 V,    │  │
         │  ├── INT1 ──┤ GPIO6     ACTIVE LOW —       │  │
         │  │  (J3.17) │           Ve out on J2.3/4)  │  │
         │  │  ┌─ CS ──┤ GPIO46 (J3.5)                │  │
         │  │  ├─ DRDY─┤ GPIO3  (J3.14)               │  │
         │  │  │       └──────────────────────────────┼──┘
         │  │  │                                      │
         ▼  ▼  │                                      │
    ┌──────────┴──┐        DIGITAL SENSOR (option A)  │
    │  ADXL355    │ VDD+VDDIO ◄── Ve ─────────────────┤
    │  (SPI accel)│ GND ── GND                        │
    └─────────────┘                                   │
         │  │  │                                      │
         ▼  ▼  ▼           ANALOG SENSOR (option B)   │
    ┌─────────────┐ DVDD/AVDD ◄── Ve ─────────────────┘
    │  ADS1220    │ GND ── GND
    │  24-bit ADC │ AIN0 ◄──┐
    │             │ AIN1 ◄──┤  differential pair (PGA ×32)
    └─────────────┘         │
                      ┌─────┴─────┐
                      │   SM-24   │  analog geophone — connects ONLY to the
                      │  geophone │  ADS1220, never to an ESP32 pin
                      └───────────┘  (~1 kΩ damping resistor across coil)
```

> ⚠️ **V4 correction:** the LoRa radio's SPI (SCK=9 / MOSI=10 / MISO=11)
> is routed **internally** to the SX1262 — those GPIOs are NOT on J2, J3,
> or the aux header (V4.3 datasheet §2.2), so sensors **cannot** share the
> radio bus on the V4 (that layout was a V3 carry-over). Sensors now use a
> **dedicated SPI bus** on header-exposed pins:
> SCK=GPIO48, MOSI=GPIO33, MISO=GPIO4.
>
> Match pins by the **GPIO numbers on the silkscreen** (47, 48, 33, 4, 6…).
> The "J2.14"-style notes are the physical position counted along the
> 18-pin header (J2 pin 1 = GND, pin 2 = 5V…; J3 pin 1 = GND, pin 2 = 3V3…)
> for boards where the silk is hard to read — GPIO number and header
> position are different numbering systems.

Wire ONE sensor option (A or B) — both share the dedicated sensor SPI bus;
only CS/INT/DRDY differ. Select with `set front_end adxl355|geophone`.

### Master pin table

Matches `platformio.ini` build flags (the source of truth).

| Function | GPIO | Dir | Notes |
|----------|------|-----|-------|
| **Radio SPI bus** (internal — no header pads; software still needs explicit pins) | | | |
| SPI SCK  | 9  | out | `SPI.begin(9, 11, 10, -1)` — explicit pins are CRITICAL (see hazards) |
| SPI MISO | 11 | in  | internal net to SX1262 only |
| SPI MOSI | 10 | out | internal net to SX1262 only |
| **LoRa SX1262** (on-board) | | | |
| NSS (CS) | 8  | out | |
| DIO1     | 14 | in  | TX-done / RX interrupt |
| RST      | 12 | out | |
| BUSY     | 13 | in  | |
| **Sensor SPI bus** (dedicated, header-exposed — both sensors) | | | |
| SCK      | 48 | out | J2, 14th position |
| MISO     | 4  | in  | J3, 15th position |
| MOSI     | 33 | out | J2, 12th position — safe on the S3R2 (quad PSRAM; only octal R8 uses GPIO33-37) |
| **ADXL355 digital accelerometer** (option A) | | | |
| CS       | 47 | out | J2, 13th position — dedicated chip-select |
| INT1     | 6  | in  | J3, 17th position — motion-wake (RTC-capable → wakes from deep sleep) |
| **ADS1220 24-bit ADC / SM-24 geophone** (option B, analog front-end) | | | |
| CS       | 46 | out | J3, 5th position — strapping pin: idles HIGH after boot, no strong pulls |
| DRDY     | 3  | in  | J3, 14th position — data-ready, polled; strapping pin, no strong pulls |
| **L76K GNSS module** | | | |
| UART RX  | 39 | in  | L76K TX → CPU (per Heltec's official V4 example) |
| UART TX  | 38 | out | CPU → L76K RX |
| EN (VGNSS_Ctrl) | 34 | out | **ACTIVE LOW** — LOW powers the GPS |
| RESET    | 42 | out | held HIGH; LOW >100 ms resets |
| **Power / misc** | | | |
| Ve sensor rail | 36 | out | ACTIVE LOW on V4 — switched 3.3 V for the sensors |
| VBAT ADC | 1  | in  | battery divider (`analogReadMilliVolts`) |
| PRG/USER button | 0 | in | active-low; debug screen |
| Solar sense | user-set | in | optional, `set solar_sense_gpio <n>` through ~100k/100k divider |

> ⚠️ **Off-limits GPIOs:**
> - **2, 7** — the V4's RF front-end amp controls (FEM_EN, VFEM_Control
>   per the V4.3 datasheet); never assign them to sensors.
> - **9, 10, 11** — LoRa SPI, internal to the SX1262, no header pads.
> - **26–32** — wired to flash/PSRAM on the ESP32-S3; touching them
>   crashes the chip (TG1WDT boot loop). (GPIO33-37 are safe on the V4's
>   S3R2 — quad PSRAM; only octal-PSRAM R8 parts claim them. GPIO35 is
>   the onboard LED.)

## Wiring the seismic sensors

Both drivers are built and selectable at runtime — no reflash needed:
`set front_end adxl355` or `set front_end geophone`, then `save`, `reboot`,
and `selftest` to verify. Both sensors share the dedicated sensor SPI bus
(SCK = GPIO48, MOSI = GPIO33, MISO = GPIO4 — match by silkscreen GPIO
number); each device gets its own chip-select.

### Option A — ADXL355 (digital, SPI) — recommended first

The sensor is the **EVAL-ADXL355-PMDZ** eval board (12-pin Pmod header:
top row 1–6, bottom row 7–12, pin 1 at the keyed/notched corner). Full
header, in PMDZ pin order:

| PMDZ pin | Signal | Wire to V4 (chip GPIO) | Header position | Notes |
|----------|--------|------------------------|-----------------|-------|
| 1  | CS         | GPIO47 | J2, 13th | dedicated chip-select |
| 2  | MOSI (SDI) | GPIO33 | J2, 12th | sensor bus |
| 3  | MISO (SDO) | GPIO4  | J3, 15th | sensor bus |
| 4  | SCLK       | GPIO48 | J2, 14th | sensor SPI clock |
| 5  | GND        | GND    | J2, 1st  | |
| 6  | VDD        | Ve rail | J2, 3rd or 4th | switched 3.3 V — powered down during sleep |
| 7  | DRDY       | — leave open | | not used by this firmware |
| 8  | INT1       | GPIO6  | J3, 17th | motion-wake interrupt (RTC-capable → wakes from deep sleep) |
| 9  | INT2       | — leave open | | |
| 10 | (NC)       | —      | | |
| 11 | GND        | (optional 2nd GND) | | |
| 12 | VDD        | (optional 2nd VDD) | | |

Seven wires total (PMDZ pins 1, 2, 3, 4, 5, 6, 8).

> ⚠️ **Breadboard trap (cost us days):** the 2×6 Pmod header shorts its
> vertical pin pairs (1/7, 2/8, **3/9**, 4/10, 5/11, 6/12) if both rows are
> seated in the same breadboard row group. A 3↔9 bridge (MISO↔INT2) is
> invisible in standby but clamps MISO low the instant measurement mode
> starts — every register read returns 0x00 and the chip looks dead, yet a
> blind standby write "revives" it. Span the breadboard's center channel or
> use jumper wires only.

> ⚠️ **Verify against the silk before soldering.** The PMDZ prints tiny
> labels (CS, MOSI, MISO, SCLK, DRDY, INT1, VDD, GND) near the header —
> match by those labels, not the pin numbers above, in case your board
> rev orders them differently. If your board has only a single populated
> 6-pin row, it carries pins 1–6 (SPI + power) but **not INT1**, which
> motion-wake needs — populate the second row.

For reference, on the **bare ADXL355 chip** (14-terminal LCC — not what
you wire to on the PMDZ): terminal 1 = CS/SCL, 2 = SCLK/VSSIO,
3 = MOSI/SDA, 4 = MISO/ASEL. The chip auto-selects SPI vs I2C: grounding
terminal 2 selects I2C (with terminal 4 as address select). The PMDZ
routes it for SPI — nothing to configure; this firmware talks SPI mode 0.

### Option B — SM-24 geophone + ADS1220 (analog front-end)

The SM-24 is an analog sensor — it never touches an ESP32 pin directly.
Its coil feeds the ADS1220's differential inputs; the ADS1220 talks SPI.

| ADS1220 pin | V4 pin | Notes |
|-------------|--------|-------|
| DVDD/AVDD   | Ve (J2, 3rd/4th) | switched 3.3 V |
| GND         | GND (J2, 1st) | |
| SCLK        | GPIO48 (J2, 14th) | sensor SPI clock |
| DIN (MOSI)  | GPIO33 (J2, 12th) | sensor bus |
| DOUT (MISO) | GPIO4 (J3, 15th)  | sensor bus |
| CS          | GPIO46 (J3, 5th)  | dedicated chip-select (strapping pin — no strong external pulls) |
| DRDY        | GPIO3 (J3, 14th)  | data-ready, polled (strapping pin — no strong external pulls) |
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
- **Auto-arm (deploy-and-forget):** on a cold boot with healthy sensor and
  radio, the node auto-enters the armed motion-wake sleep cycle after
  `auto_arm_s` seconds (default 120) of an untouched bench CLI. Any
  keystroke or PRG press cancels it for the session; `set auto_arm_s 0`
  disables it entirely. So in the field: power it up, walk away, and two
  minutes later it's a trip sensor.

## WiFi radar (CSI) sensing

A second, RF-based sensing channel alongside the seismic front end
(modeled on espressif/esp-csi and espectre). The ESP32-S3's WiFi radio
captures Channel State Information from every frame it decodes; a person
moving near the TX→RX path perturbs the per-subcarrier amplitudes, and an
on-device detector turns that into `wifi_presence` alerts over the normal
encrypted LoRa path. It sees people through walls and vegetation where
ground coupling is poor, and both channels can fire on one node — a
correlated seismic + wifi_presence pair is a strong detection.

**Build & runtime gates.** `-DSPS_CSI_ENABLE=1` in `platformio.ini`
compiles the module in (set to 0 to strip WiFi entirely for deep-sleep
deployments); `set csi_enable 1` + `save` + `reboot` turns it on. CSI
keeps the WiFi radio in RX continuously, so an enabled node **stays in
continuous-listen mode and never deep sleeps** (auto-arm is suppressed) —
treat it like the geophone profile: mains/solar power only.

**Traffic source.** CSI measures received frames, so something must
transmit. Each node sends tiny ESP-NOW broadcast pings (`csi_ping_hz`,
default 10 Hz, ~8-byte payload ≈ well under 0.5 % duty cycle); any pair
of nodes on the same `csi_wifi_channel` forms a TX→RX sensing link with
no router around. Roles: `set csi_role rx|tx|both` (default both).
Ambient AP/router traffic on the configured channel is captured too — in
a home/urban deployment an `rx`-only node can ride on router beacons
(~10 Hz on their own) with zero TX power spent.

**Detector (espectre-style).** Per frame: spatial turbulence =
std/mean of mid-subcarrier amplitudes (scale-free, cancels AGC/RSSI).
Then a moving variance over `csi_window_frames` (default 64), divided by
a baseline learned during `csi_calib_s` (default 30 s — keep the area
clear after boot). Trigger at metric ≥ `csi_threshold` (default 2.0),
release at 0.5×, then `csi_holdoff_s` (default 5 s) of hold-off, so a
passing person yields one clean event. Heartbeats report the quiescent
baseline (×100) plus CSI-on/calibrating health flags, so the gateway can
watch the RF noise floor per node.

**Tuning.** `csi` on the CLI prints status; `csi 30` streams the live
metric for 30 s — walk the perimeter and pick a threshold ~2× the largest
quiet-time metric you see. Raise `csi_window_frames` for fewer false
positives (slower response), raise `csi_ping_hz` for faster response
(more power). If two CSI nodes are in radio range, put them on the same
channel and let one be `tx`/`both` — the *link between them* is the
sensitive zone.

**Coexistence & power.** LoRa (SX1262, SPI) and WiFi are separate radios
on this board, so there's no shared-RF arbitration; the firmware still
pauses ESP-NOW pings while an alert burst + ACK window is in flight so
the two transmitters never key up together (peak-current precaution on
battery). Expected cost: WiFi RX-always-on adds roughly 80–100 mA
continuous (vs ~40 mA continuous-listen without CSI, vs ~20 µA armed
deep sleep) — bench-verify on real hardware and record actuals here.

**Config keys** (all `set`-able, see `help`): `csi_enable`, `csi_role`,
`csi_wifi_channel` (1–13, must match across sensing nodes), `csi_ping_hz`
(1–100), `csi_threshold` (>1.1), `csi_window_frames` (8–128),
`csi_calib_s` (5–600), `csi_holdoff_s` (0–600).

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
