#!/bin/bash
set -euo pipefail

SERVICE_NAME=${SERVICE_NAME:-pyro.service}

sudo systemctl daemon-reload
sudo systemctl restart "$SERVICE_NAME"
sudo systemctl --no-pager status "$SERVICE_NAME"
echo
echo "Follow logs with: sudo journalctl -u $SERVICE_NAME -f"
