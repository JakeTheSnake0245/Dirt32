#!/usr/bin/env bash
# Dirt32 gateway installer — Raspberry Pi / Ubuntu LTS.
# Run from the repo root:  sudo bash linux/install.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then echo "run with sudo"; exit 1; fi
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== packages (python3 is stdlib-only; mosquitto is the MQTT broker) =="
apt-get update -qq
apt-get install -y -qq python3 mosquitto

echo "== user + directories =="
id -u dirt32 &>/dev/null || useradd --system --home /var/lib/dirt32 dirt32
usermod -aG dialout dirt32          # serial port access
mkdir -p /opt/dirt32 /var/lib/dirt32 /etc/dirt32
cp -r "$HERE/gatewayd" /opt/dirt32/
chown -R dirt32:dirt32 /var/lib/dirt32

echo "== config =="
if [[ ! -f /etc/dirt32/gateway.json ]]; then
    cp "$HERE/gateway.example.json" /etc/dirt32/gateway.json
    sed -i 's#"keys_path": .*#"keys_path": "/etc/dirt32/keys.json",#' /etc/dirt32/gateway.json
    sed -i 's#"db_path": .*#"db_path": "/var/lib/dirt32/dirt32.sqlite3",#' /etc/dirt32/gateway.json
    echo "   wrote /etc/dirt32/gateway.json — EDIT net_id + serial_device"
fi
if [[ ! -f /etc/dirt32/keys.json ]]; then
    echo '{}' > /etc/dirt32/keys.json
    chmod 600 /etc/dirt32/keys.json
    chown dirt32:dirt32 /etc/dirt32/keys.json
    echo "   wrote empty /etc/dirt32/keys.json — add node keys (see linux/README.md)"
fi

echo "== mosquitto (localhost-only by default — see README to open to the LAN) =="
cat > /etc/mosquitto/conf.d/dirt32.conf <<'EOF'
# Dirt32 default: broker only reachable from the Pi itself.
# To let other machines subscribe, add an authenticated listener —
# see 'MQTT' in linux/README.md. Do NOT just set allow_anonymous true.
listener 1883 127.0.0.1
allow_anonymous true
EOF
systemctl enable --now mosquitto

echo "== service =="
cp "$HERE/systemd/dirt32-gateway.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable dirt32-gateway
# 'enable --now' does NOT restart an already-running service, which left the
# old code running after upgrades — always restart so new code takes effect.
systemctl restart dirt32-gateway

echo
echo "Done. Web GUI: http://$(hostname -I | awk '{print $1}'):9000"
echo "MQTT:          mosquitto_sub -t 'dirt32/#' -v"
echo "Logs:          journalctl -u dirt32-gateway -f"
