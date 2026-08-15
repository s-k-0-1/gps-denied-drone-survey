#!/usr/bin/env bash
# Launch the Drone Base Station dashboard (run from anywhere).
set -e
cd "$(dirname "$0")/.."          # → ~/advanced_matcher (pipeline root)

# Optional knobs (uncomment / edit as needed):
# export DRONE_LINK_MODE=simulator        # force simulator, no hardware
# export DRONE_LINK_MODE=mavlink          # try real drone, fall back to sim (default)
# export MAVLINK_CONN="udpin:0.0.0.0:14550"
# export BASE_STATION_PORT=8000
# export RUN_PIPELINE_ON_MISSION=0        # display-only by default
# export BASE_STATION_HOST=0.0.0.0        # expose to the LAN (default is loopback only)
# export IROC_USER=me IROC_PASS='...'     # otherwise generated on first run

PORT="${BASE_STATION_PORT:-8000}"
echo "──────────────────────────────────────────────────────────"
echo "  Drone Base Station  →  http://localhost:${PORT}"
echo "  Link mode: ${DRONE_LINK_MODE:-simulator}   (sim fallback always on)"
echo "──────────────────────────────────────────────────────────"

exec python3 -m base_station.server
