#!/bin/bash
set -euo pipefail

SERVICE_NAME=${SERVICE_NAME:-pyro.service}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE_SOURCE="$SCRIPT_DIR/pyro.service"
SERVICE_TARGET="/etc/systemd/system/$SERVICE_NAME"

if [[ -f "$SERVICE_SOURCE" ]]; then
  echo "Updating $SERVICE_NAME from $SERVICE_SOURCE"
  sudo cp "$SERVICE_SOURCE" "$SERVICE_TARGET"
fi

sudo systemctl daemon-reload
sudo systemctl restart "$SERVICE_NAME"
sudo systemctl --no-pager status "$SERVICE_NAME"
echo
echo "Follow logs with: sudo journalctl -u $SERVICE_NAME -f"
