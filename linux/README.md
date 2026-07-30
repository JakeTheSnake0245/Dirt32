# Dirt32 — Linux gateway (Raspberry Pi, Ubuntu LTS)

The trust boundary and control room of the perimeter: a Heltec board on USB
receives LoRa frames as a dumb radio bridge, and this daemon verifies,
stores, publishes, and displays them.

```
[nodes] ~~LoRa 903 MHz~~> [bridge ESP32, USB] --serial--> dirt32-gateway (this)
                                                          ├── SQLite (alerts, heartbeats, raw frames)
                                                          ├── MQTT  -> mosquitto -> your other systems
                                                          └── Web GUI (blackbox map, live feed)
```

**All keys live on the Pi.** The bridge never decrypts anything — a stolen
bridge board contains nothing secret. The daemon verifies every frame
(ChaCha20-Poly1305, per-node key), enforces the replay window (persisted
across restarts), stores everything (including forged/replayed frames, for
forensics), publishes to MQTT, and seals + transmits ACKs back to nodes.

The daemon is **pure Python stdlib** — no pip packages needed on the Pi.

## Install (on the Pi)

```sh
git clone <your repo> && cd Dirt32
sudo bash linux/install.sh
```

The installer sets up: python3 + mosquitto, a `dirt32` system user, the
daemon under systemd (`dirt32-gateway`), config in `/etc/dirt32/`, and data
in `/var/lib/dirt32/`.

Then:

1. Flash the bridge board: `cd esp32/gateway-bridge && pio run -t upload`,
   plug it into the Pi.
2. Edit `/etc/dirt32/gateway.json` — set `net_id` and the radio params to
   match your nodes, and `serial_device` (usually `/dev/ttyUSB0`).
3. Register node keys in `/etc/dirt32/keys.json` — node id → the 64-hex
   PSK shown by `keygen` on the node:
   ```json
   { "1": "a1b2...64 hex...", "2": "..." }
   ```
4. `sudo systemctl restart dirt32-gateway`

## Web GUI — the blackbox map

`http://<pi-ip>:9000`

Dark map with one dot per sensor:

- **GREEN** — heartbeats arriving as expected.
- **YELLOW** — no heartbeat for half a day, low battery, or a health fault
  (sensor/self-test failure).
- **RED** (pulsing) — tamper flag, node moved farther than expected from
  its provisioned position, or silent for a whole day.

Hover a dot for details (reasons, battery, RSSI, last seen); the sidebar
lists every sensor and a live feed of alerts/heartbeats. Thresholds are
the `status` block in `gateway.json`.

## MQTT — for your other systems

The daemon publishes JSON to the local mosquitto broker. **By default the
broker only listens on localhost** (on the Pi itself):

```sh
mosquitto_sub -t 'dirt32/#' -v      # on the Pi
```

To let other machines subscribe, add an authenticated LAN listener:

```sh
sudo mosquitto_passwd -c /etc/mosquitto/passwd subscriber
```

then in `/etc/mosquitto/conf.d/dirt32.conf` add:

```
listener 1883 0.0.0.0
allow_anonymous false
password_file /etc/mosquitto/passwd
```

and `sudo systemctl restart mosquitto`. Then from anywhere on the LAN:

```sh
mosquitto_sub -h <pi-ip> -u subscriber -P <password> -t 'dirt32/#' -v
```

| Topic | Payload | Retained |
|---|---|---|
| `dirt32/alert/<node_id>` | event, confidence, peak, battery, rssi/snr | no |
| `dirt32/heartbeat/<node_id>` | battery, position, health flags, noise | no |
| `dirt32/status/<node_id>` | `{"color": "green|yellow|red", "reasons": [...]}` | **yes** — subscribers get current state immediately |
| `dirt32/gateway/state` | `{"online": true/false}` (LWT) | yes |

