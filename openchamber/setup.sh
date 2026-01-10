#!/bin/bash

# 스크립트 디렉토리로 이동
cd "$(dirname "$0")"

echo ">> Installing openchamber..."
./install.sh

if [ $? -ne 0 ]; then
    echo "Installation failed. Aborting service setup."
    exit 1
fi

echo ">> Resolving executable paths..."

# Find the executable as the CURRENT user
TARGET_EXEC=$(which openchamber)

if [ -z "$TARGET_EXEC" ]; then
    echo "Error: 'openchamber' not found in PATH after installation."
    exit 1
fi

# Find the node executable directory to ensure systemd finds 'node'
NODE_EXEC=$(which node)
NODE_BIN_DIR=""
if [ -n "$NODE_EXEC" ]; then
    NODE_BIN_DIR=$(dirname "$NODE_EXEC")
fi

echo "   OpenChamber: $TARGET_EXEC"
echo "   Node Bin   : $NODE_BIN_DIR"

echo ">> Setting up openchamber systemd service..."
# Pass paths and user to the service installer
# We use sudo here to elevate privileges for service creation
sudo ./install-service.sh "$TARGET_EXEC" "$USER" "$NODE_BIN_DIR"
