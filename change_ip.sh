#!/usr/bin/env bash
# ============================================================================
#  Change the ground-station (PC) IP everywhere, in one command.
#
#  USAGE (run on the PC, inside the project folder):
#      ./change_ip.sh 192.168.1.100
#
#  Find your current PC IP with:   hostname -I
#
#  It updates BASE_URL / PC_IP in:
#     1. esp32_firmware/full_base_station_wifi.ino
#     2. jetson/landing_transfer_notify.py          (if present)
#  and prints the remaining manual steps (re-flash the ESP, update the Jetson,
#  edit mavlink-router).
# ============================================================================
set -e

NEW="$1"
if [ -z "$NEW" ]; then
  echo "Usage: $0 <new-pc-ip>"
  echo "  e.g. $0 192.168.1.100      (find it with:  hostname -I)"
  exit 1
fi

# basic IP sanity check
if ! echo "$NEW" | grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$'; then
  echo "ERROR: '$NEW' is not a valid IPv4 address."
  exit 1
fi

ROOT="$(cd "$(dirname "$0")" && pwd)"
INO="$ROOT/esp32_firmware/full_base_station_wifi.ino"
JET="$ROOT/jetson/landing_transfer_notify.py"

[ -f "$INO" ] || { echo "ERROR: not found: $INO"; exit 1; }

echo "Setting ground-station IP to: $NEW"
echo

# 1) ESP firmware  ->  BASE_URL = "http://<ip>:8000"
sed -i -E "s|(BASE_URL[[:space:]]*=[[:space:]]*\"http://)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(:8000\")|\1$NEW\2|" "$INO"

# 2) Jetson script (optional) ->  PC_IP  and  BASE_URL
if [ -f "$JET" ]; then
  sed -i -E "s|(PC_IP[[:space:]]*=[[:space:]]*\")[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(\")|\1$NEW\2|" "$JET"
  sed -i -E "s|(BASE_URL[[:space:]]*=[[:space:]]*\"http://)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(:8000\")|\1$NEW\2|" "$JET"
else
  echo "(note: $JET not present — skipping the Jetson script)"
fi

echo "Updated:"
grep -n "$NEW" "$INO" || true
if [ -f "$JET" ]; then
  grep -n "$NEW" "$JET" || true
fi

cat <<EOF

──────────────────────────────────────────────────────────────
NEXT STEPS (to make it live)
──────────────────────────────────────────────────────────────
1. RE-FLASH THE ESP32 with:
     $INO

2. ON THE JETSON — update the data-transfer script (if you use one):
     scp <this-file> <jetson-user>@<jetson-ip>:~/scripts/
     sudo systemctl restart landing-transfer.service

3. ON THE JETSON — update mavlink-router (so QGroundControl reaches the PC):
     sudo nano /etc/mavlink-router/main.conf
       [UdpEndpoint QGC]  ->  Address = $NEW
     sudo systemctl restart mavlink-router

4. VERIFY (from the Jetson):
     curl -i -X POST -H "X-Auth-Token: <YOUR_TOKEN>" "http://$NEW:8000/api/landed"   # expect 200
──────────────────────────────────────────────────────────────
EOF
