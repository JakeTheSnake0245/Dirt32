# Dirt32 — Distributed Seismic Perimeter Sensor System

Firmware + protocol stack for a LoRa-linked buried seismic sensor network (nodes → relays → gateway → portal). Authoritative spec: `attached_assets/Pasted--Distributed-Seismic-Perimeter-Sensor-System-*.txt`.

## Firmware (current focus)

- `esp32/lib/sps_proto` — shared C99 wire protocol + ChaCha20-Poly1305 AEAD, replay window, relay dedupe. Reused verbatim by node, relay, and gateway tiers.
- `esp32/node` — PlatformIO project for the Heltec V4 sensor node (front-end abstraction ADXL355/geophone, STA/LTA detector, LoRa TX + closed-loop ACK, NVS JSON config + serial CLI).
- `esp32/gateway-bridge` — dumb radio↔USB-serial bridge flashed on a Heltec plugged into the Pi. Holds NO keys; line protocol `RX <hex> <rssi> <snr>` / `TX <hex>` / `CFG ...`.
- Host tests: `cd esp32/test/host && gcc ... && ./sps_test` (see `esp32/README.md`).
- Hardware flashing happens on the user's machine via `pio run -t upload`; it cannot be done from this workspace.

## Linux gateway (Raspberry Pi / Ubuntu LTS)

- `linux/gatewayd/dirt32_gateway/` — pure-stdlib Python daemon: `proto.py` (byte-for-byte port of sps_proto, cross-verified against C vectors), `ingest.py` (trust boundary: decrypt → replay check → store → MQTT → ACK), `db.py` (SQLite), `mqtt.py` (minimal MQTT 3.1.1 publisher to mosquitto), `serial_link.py` (termios, no pyserial), `web.py` + `static/index.html` (dark map GUI, endpoints under `/gw/`), `simulator.py`.
- Tests: `cd linux/gatewayd && python3 tests/test_proto.py` — includes C-sealed vectors (regenerate via `esp32/test/host/make_vectors.c`). This is the node↔gateway compatibility contract.
- Install on the Pi: `sudo bash linux/install.sh` (systemd unit `dirt32-gateway`, mosquitto broker, config in `/etc/dirt32/`).
- `artifacts/gateway-demo` — workspace preview only: runs the real daemon in simulator mode (its `dev` script calls `linux/gatewayd/demo_run.py`). Not part of the Pi deployment.
- Status colors: green = heartbeats OK; yellow = silent >12 h / battery <3300 mV / sensor-selftest fault; red = tamper, moved >30 m from provisioned position, or silent >24 h. Thresholds in `gateway.json` `status` block.

## Run & Operate

- `pnpm --filter @workspace/api-server run dev` — run the API server (port 5000)
- `pnpm run typecheck` — full typecheck across all packages
- `pnpm run build` — typecheck + build all packages
- `pnpm --filter @workspace/api-spec run codegen` — regenerate API hooks and Zod schemas from the OpenAPI spec
- `pnpm --filter @workspace/db run push` — push DB schema changes (dev only)
- Required env: `DATABASE_URL` — Postgres connection string

## Stack

- pnpm workspaces, Node.js 24, TypeScript 5.9
- API: Express 5
- DB: PostgreSQL + Drizzle ORM
- Validation: Zod (`zod/v4`), `drizzle-zod`
- API codegen: Orval (from OpenAPI spec)
- Build: esbuild (CJS bundle)

## Where things live

_Populate as you build — short repo map plus pointers to the source-of-truth file for DB schema, API contracts, theme files, etc._

## Architecture decisions

_Populate as you build — non-obvious choices a reader couldn't infer from the code (3-5 bullets)._

## Product

_Describe the high-level user-facing capabilities of this app once they exist._

## User preferences

- Hardware in hand (July 2026): Heltec LoRa V4 boards + Heltec L76K GNSS modules; no seismic sensors yet — bench verification via `taptest` CLI.

## Gotchas

_Populate as you build — sharp edges, "always run X before Y" rules._

## Pointers

- See the `pnpm-workspace` skill for workspace structure, TypeScript setup, and package details
