#!/bin/bash
set -euo pipefail

REPO_DIR=${REPO_DIR:-/home/pyro/pyro}
VENV_PATH=${VENV_PATH:-/home/pyro/venv}

cd "$REPO_DIR"
source "$VENV_PATH/bin/activate"

exec python3 main.py
