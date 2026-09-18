#!/bin/zsh
set -u

VIEWER_DIR="${0:A:h}"
VIEWER_URL="http://127.0.0.1:8765"

cd "$VIEWER_DIR" || exit 1

if ! /usr/bin/curl --silent --fail "$VIEWER_URL" >/dev/null 2>&1; then
  /usr/bin/python3 -m http.server 8765 --bind 127.0.0.1 >/tmp/ranger-radar-viewer.log 2>&1 &
  SERVER_PID=$!
  trap 'kill "$SERVER_PID" >/dev/null 2>&1 || true' EXIT INT TERM
  sleep 1
fi

if [[ -d "/Applications/Google Chrome.app" ]]; then
  open -a "Google Chrome" "$VIEWER_URL"
else
  open "$VIEWER_URL"
fi

echo "RANGER Radar Viewer is running at $VIEWER_URL"
echo "Keep this window open while using the viewer. Press Control-C to stop it."
wait
