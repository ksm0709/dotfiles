#!/bin/bash
#
# install-service.sh — Register openchamber as a systemd service via bun
#
# bun을 사용하므로 NVM/Node PATH 탐색이 불필요합니다.
#
# Usage: sudo ./install-service.sh [service_user] [port]
#
# Arguments:
#   $1: User to run the service as (optional, defaults to SUDO_USER)
#   $2: Port to listen on (optional, defaults to 8888)

set -euo pipefail

SERVICE_USER="${1:-${SUDO_USER:-}}"
PORT="${2:-8888}"
SERVICE_FILE="/etc/systemd/system/openchamber.service"

# ── Root check ────────────────────────────────────────────────────────────
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    exit 1
fi

# ── Validate service user ─────────────────────────────────────────────────
if [ -z "$SERVICE_USER" ]; then
    echo "Error: No service user specified and SUDO_USER not found."
    echo "Usage: sudo ./install-service.sh [username] [port]"
    exit 1
fi

if ! id "$SERVICE_USER" &>/dev/null; then
    echo "Error: User '$SERVICE_USER' does not exist."
    exit 1
fi

if [ "$SERVICE_USER" = "root" ]; then
    echo "Warning: Running service as root is not recommended."
fi

USER_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)

# ── Locate bun ────────────────────────────────────────────────────────────
BUN_PATH="$USER_HOME/.bun/bin/bun"
if [ ! -x "$BUN_PATH" ]; then
    BUN_PATH=$(command -v bun 2>/dev/null || true)
fi

if [ -z "$BUN_PATH" ] || [ ! -x "$BUN_PATH" ]; then
    echo "Error: bun not found for user '$SERVICE_USER'."
    echo "  Expected : $USER_HOME/.bun/bin/bun"
    echo "  Install  : curl -fsSL https://bun.sh/install | bash"
    exit 1
fi

# ── Locate @openchamber/web package ──────────────────────────────────────
# NVM 환경을 고려해 서비스 유저 컨텍스트에서 npm root를 조회합니다.
NPM_GLOBAL_ROOT=$(sudo -u "$SERVICE_USER" bash -c '
    if [ -d "$HOME/.nvm" ]; then
        export NVM_DIR="$HOME/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
    fi
    npm root -g 2>/dev/null
' 2>/dev/null)

PACKAGE_ROOT="$NPM_GLOBAL_ROOT/@openchamber/web"
PACKAGE_JSON="$PACKAGE_ROOT/package.json"

if [ ! -f "$PACKAGE_JSON" ]; then
    echo "Error: @openchamber/web package not found at $PACKAGE_ROOT"
    echo "  Run './install.sh' first to install the package."
    exit 1
fi

# package.json에서 main entry point를 추출합니다.
# python3는 대부분의 Linux 환경에서 root로도 사용 가능합니다.
MAIN_ENTRY=$(python3 -c "
import json, sys
with open('$PACKAGE_JSON') as f:
    d = json.load(f)
print(d.get('main', d.get('module', 'index.js')))" 2>/dev/null || echo "index.js")

EXEC_MAIN="$PACKAGE_ROOT/$MAIN_ENTRY"

if [ ! -f "$EXEC_MAIN" ]; then
    echo "Error: Entry point not found: $EXEC_MAIN"
    echo "  package.json 'main' field resolved to: $MAIN_ENTRY"
    echo "  Verify the package is correctly installed."
    exit 1
fi

# ── Write systemd service file ────────────────────────────────────────────
echo "Configuring systemd service..."
echo "  Service : $SERVICE_FILE"
echo "  User    : $SERVICE_USER"
echo "  Bun     : $BUN_PATH"
echo "  Entry   : $EXEC_MAIN"
echo "  Port    : $PORT"

cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=OpenChamber Service
After=network.target

[Service]
Type=simple
User=$SERVICE_USER
Environment=HOME=$USER_HOME
ExecStart=$BUN_PATH run $EXEC_MAIN --port $PORT
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# ── Enable and start ──────────────────────────────────────────────────────
echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling openchamber service..."
systemctl enable openchamber

echo "Starting openchamber service..."
systemctl start openchamber

echo ""
systemctl status openchamber --no-pager
