---
name: Dirt32 Linux gateway decisions
description: Non-obvious architecture decisions for the Pi gateway + bridge tier
---

- **Bridge holds no keys.** The ESP32 on the Pi's USB port is a dumb radio↔serial bridge (`RX <hex> <rssi> <snr>` / `TX <hex>` / `CFG ...`). All crypto/trust lives in the Python daemon.
  **Why:** a stolen or tapped bridge board must reveal nothing; also keeps bridge firmware trivial.
  **How to apply:** never add decryption, key storage, or filtering beyond framing to the bridge firmware.
- **Gateway daemon is pure Python stdlib** (own ChaCha20-Poly1305, raw-socket MQTT 3.1.1 pub, termios serial, http.server GUI).
  **Why:** must run on a stock Ubuntu Pi with no pip; user wanted single-command install.
  **How to apply:** don't add pip dependencies to `linux/gatewayd`; mosquitto (apt) handles MQTT fan-out/auth.
- **Python↔C compatibility contract** = `linux/gatewayd/tests/test_proto.py` with vectors from `esp32/test/host/make_vectors.c`. Any wire-format change requires regenerating vectors and passing both suites; gateway-sealed ACKs must be byte-identical to C-sealed ones.
- **Gateway web endpoints live under `/gw/`, not `/api/`** — in the Replit workspace preview, `/api/*` is captured by the shared api-server artifact's proxy route.
- Status colors (user-specified): green = heartbeats OK; yellow = silent >12 h, battery low, sensor/self-test fault; red = tamper, moved > threshold from provisioned position, silent >24 h. Configurable in `gateway.json`.
- V4 FEM pin constraint: GPIO 2/5/7/46 are used by the Heltec V4's RF front-end — never assign them to sensors.
