#!/usr/bin/env bash
# Build the firmware, serve the image from this host and ask the robot to pull
# it over Wi-Fi (OTA). No USB cable required.
#
#   tools/ota.sh                       # uses defaults below
#   ROBOT=192.168.1.195 tools/ota.sh
set -euo pipefail

ROBOT="${ROBOT:-192.168.1.195}"
PORT="${PORT:-8000}"
ACTIVATE="${IDF_ACTIVATE:-$HOME/.espressif/tools/activate_idf_v6.1.sh}"
cd "$(dirname "$0")/.."

# The IDF activation script refuses to be sourced from a script (it inspects
# $0), so build inside a `bash -c` whose $0 is "bash".
build_firmware() {
  if command -v idf.py >/dev/null 2>&1; then
    idf.py build
  else
    bash -c 'source "$1" >/dev/null 2>&1 || true; idf.py build' bash "$ACTIVATE"
  fi
}
build_firmware

HOST_IP="${HOST_IP:-$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null)}"
if [ -z "$HOST_IP" ]; then
  echo "could not determine host IP; set HOST_IP=..." >&2
  exit 1
fi

echo "serving build/ on http://$HOST_IP:$PORT"
( cd build && exec python3 -m http.server "$PORT" ) >/tmp/wlrobot_ota_server.log 2>&1 &
SERVER_PID=$!
trap 'kill "$SERVER_PID" 2>/dev/null || true' EXIT
sleep 1

URL="http://$HOST_IP:$PORT/micro_wheeled_leg_bot.bin"
uptime() {
  curl -s --max-time 2 "http://$ROBOT/api/status" | \
    python3 -c 'import sys,json;print(json.load(sys.stdin)["uptime"])' 2>/dev/null || true
}
BEFORE="$(uptime)"
echo "triggering OTA on $ROBOT: $URL"
curl -s -X POST --data "$URL" "http://$ROBOT/api/ota"
echo

echo "waiting for the robot to reboot..."
for _ in $(seq 1 60); do
  sleep 1
  NOW="$(uptime)"
  if [ -n "$NOW" ] && [ -n "$BEFORE" ] && [ "$BEFORE" -gt 0 ] && [ "$NOW" -lt "$BEFORE" ]; then
    echo "robot rebooted into the new image (uptime ${NOW}s)"
    exit 0
  fi
done
echo "robot did not reboot; check /tmp/wlrobot_ota_server.log" >&2
exit 1
