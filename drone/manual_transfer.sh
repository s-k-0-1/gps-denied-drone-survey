#!/usr/bin/env bash
# ============================================================================
#  MANUAL data transfer — rsync the latest survey folder to the PC.
#  Use this if the AUTOMATIC (landing-triggered) transfer didn't run.
#
#  USAGE (on the Jetson):
#      ./manual_transfer.sh                 # uses the default PC IP below
#      ./manual_transfer.sh 10.56.178.123   # or pass the PC IP
#
#  Get the PC IP on the PC with:  hostname -I
#  Safe: NO --delete, so it only ADDS/updates photos, deletes nothing.
# ============================================================================
set -e

PC_IP="${1:-10.56.178.123}"          # <- default PC IP (override by passing an arg)
PC_USER="sachin"
PC_PORT="2222"
PC_DEST="/home/sachin/advanced_matcher/drone_photos/"
SURVEY_ROOT="/media/jetson/ROS2_SSD/survey/"

# find the newest survey folder
LATEST="$(ls -dt ${SURVEY_ROOT}*survey_* 2>/dev/null | head -1)"
if [ -z "$LATEST" ]; then
  echo "ERROR: no survey folder found under $SURVEY_ROOT"
  exit 1
fi

echo "Latest survey : $LATEST"
echo "Sending to    : $PC_USER@$PC_IP:$PC_DEST"
echo

rsync -avz --partial -e "ssh -p $PC_PORT" "$LATEST/" "$PC_USER@$PC_IP:$PC_DEST"

echo
echo "✅ Transfer complete: $(basename "$LATEST")"
echo "   (optional) also start docking:  curl -X POST -H \"X-Auth-Token: \$DOCK_TOKEN\" \"http://$PC_IP:8000/api/landed\""
