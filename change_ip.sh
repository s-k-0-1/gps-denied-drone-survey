#!/usr/bin/env bash
# ============================================================================
#  LUMA — change the PC (base-station) IP everywhere, in one command.
#
#  USAGE (run on the PC, inside ~/advanced_matcher):
#      ./change_ip.sh 10.56.178.123
#
#  Get your current PC IP first with:   hostname -I
#
#  It updates:
#     1. esp32_firmware/full_base_station_wifi.ino   -> BASE_URL
#     2. jetson/landing_transfer_notify.py           -> PC_IP + BASE_URL
#  and then prints the remaining manual steps (ESP re-flash, Jetson copy,
#  mavlink-router edit).
# ============================================================================
set -e

NEW="$1"
if [ -z "$NEW" ]; then
  echo "Usage: $0 <new-pc-ip>"
  echo "  e.g. $0 10.56.178.123      (find it with:  hostname -I)"
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

for f in "$INO" "$JET"; do
  [ -f "$f" ] || { echo "ERROR: not found: $f"; exit 1; }
done

echo "Setting base-station IP to: $NEW"
echo

# 1) ESP firmware  ->  BASE_URL = "http://<ip>:8000"
sed -i -E "s|(BASE_URL[[:space:]]*=[[:space:]]*\"http://)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(:8000\")|\1$NEW\2|" "$INO"

# 2) Jetson script ->  PC_IP = "<ip>"   and   BASE_URL = "http://<ip>:8000"
sed -i -E "s|(PC_IP[[:space:]]*=[[:space:]]*\")[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(\")|\1$NEW\2|" "$JET"
sed -i -E "s|(BASE_URL[[:space:]]*=[[:space:]]*\"http://)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(:8000\")|\1$NEW\2|" "$JET"

echo "Updated:"
grep -n "$NEW" "$INO" "$JET" || true

cat <<EOF

──────────────────────────────────────────────────────────────
NEXT STEPS (do these to make it live)
──────────────────────────────────────────────────────────────
1. RE-FLASH THE ESP with:
     $INO

2. ON THE JETSON — update the data-transfer script:
     scp -P 2222 sachin@$NEW:$JET \\
         /home/jetson/scripts/landing_transfer_node.py
     sudo systemctl restart landing-transfer.service

3. ON THE JETSON — update mavlink-router (for QGC):
     sudo nano /etc/mavlink-router/main.conf
       [UdpEndpoint QGC]  ->  Address = $NEW
     sudo pkill -f mavlink-routerd
     sudo systemctl restart mavlink-router

4. VERIFY (from the Jetson):
     curl -i -X POST "http://$NEW:8000/api/landed?token=lumadock"    # expect 200
     ssh -p 2222 sachin@$NEW "echo ok"                               # rsync path
──────────────────────────────────────────────────────────────
EOF
