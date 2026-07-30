# Distributed Seismic Perimeter Sensor System

Firmware + protocol stack for a LoRa-linked buried seismic sensor network (nodes → relays → gateway → portal). Authoritative spec: `attached_assets/Pasted--Distributed-Seismic-Perimeter-Sensor-System-*.txt`.

## Firmware (current focus)

- `firmware/lib/sps_proto` — shared C99 wire protocol + ChaCha20-Poly1305 AEAD, replay window, relay dedupe. Reused verbatim by node, relay, and gateway tiers.
- `firmware/node` — PlatformIO project for the Heltec V4 sensor node (front-end abstraction ADXL355/geophone, STA/LTA detector, LoRa TX + closed-loop ACK, NVS JSON config + serial CLI).
- Host tests: `cd firmware/test/host && gcc ... && ./sps_test` (see `firmware/README.md`).
- Hardware flashing happens on the user's machine via `pio run -t upload`; it cannot be done from this workspace.

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

_Populate as you build — explicit user instructions worth remembering across sessions._

## Gotchas

_Populate as you build — sharp edges, "always run X before Y" rules._

## Pointers

- See the `pnpm-workspace` skill for workspace structure, TypeScript setup, and package details
