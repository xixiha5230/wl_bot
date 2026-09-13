#!/usr/bin/env bash
# Serve the local control UI. Open the printed URL in a browser.
set -euo pipefail
cd "$(dirname "$0")/web"
PORT="${PORT:-8080}"
IP="$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || echo 127.0.0.1)"
echo "WLROBOT UI: http://$IP:$PORT/  (or http://localhost:$PORT/)"
exec python3 -m http.server "$PORT"
