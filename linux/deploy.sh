#!/usr/bin/env bash
# Dirt32 gateway update script.
# Run after every `git pull` to push new daemon code into the running service.
#
#   cd ~/Dirt32 && git pull && sudo bash linux/deploy.sh
#
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== copying daemon files =="
cp -r "$HERE/gatewayd/dirt32_gateway" /opt/dirt32/gatewayd/

echo "== restarting service =="
systemctl restart dirt32-gateway
sleep 2
systemctl status dirt32-gateway --no-pager | head -8

echo ""
echo "Done. Tail logs:  journalctl -u dirt32-gateway -f"
