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

# Check for NVM environment (common source of path issues)
if [ -n "$NVM_DIR" ] || [ -d "$HOME/.nvm" ]; then
    echo "   Detected NVM environment."
    echo "   Note: Ensure NVM is properly loaded for the service user."
fi

# Find the executable as the CURRENT user
TARGET_EXEC=$(which openchamber 2>/dev/null)

# If not found, try to find hash-suffixed version (e.g., .openchamber-Lr8RUQ7B)
# This handles npm's behavior of creating hashed binaries for some packages
if [ -z "$TARGET_EXEC" ]; then
    echo "   Standard 'openchamber' not found. Searching for alternative names..."
    NODE_BIN_DIR=$(dirname "$(which node 2>/dev/null)" 2>/dev/null)
    if [ -n "$NODE_BIN_DIR" ]; then
        # Look for .openchamber-* pattern
        HASHED_EXEC=$(ls -1 "$NODE_BIN_DIR"/.openchamber-* 2>/dev/null | head -1)
        if [ -n "$HASHED_EXEC" ]; then
            echo "   Found hash-suffixed executable: $HASHED_EXEC"
            TARGET_EXEC="$HASHED_EXEC"
        fi
    fi
fi

if [ -z "$TARGET_EXEC" ]; then
    echo "Error: 'openchamber' not found in PATH after installation."
    echo "   Please check that the installation completed successfully."
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

# Verify the executable is valid
if [ ! -x "$TARGET_EXEC" ]; then
    echo "Error: Found openchamber at '$TARGET_EXEC' but it is not executable."
    exit 1
fi

# Warn if running without sudo
if [ "$EUID" -eq 0 ]; then
    echo "Warning: Running as root. The service will be created for root user."
fi

echo ">> Setting up openchamber systemd service..."
# Pass paths and user to the service installer
# We use sudo here to elevate privileges for service creation
sudo ./install-service.sh "$TARGET_EXEC" "$USER" "$NODE_BIN_DIR"
