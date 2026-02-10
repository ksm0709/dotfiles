#!/bin/bash

if ! command -v npm &> /dev/null; then
    echo "Error: npm is not installed. Please install Node.js and npm first."
    exit 1
fi

# Check if npm global prefix is writable
NPM_PREFIX=$(npm config get prefix)
if [ ! -w "$NPM_PREFIX" ] && [ "$EUID" -ne 0 ]; then
    echo "Error: npm global prefix ($NPM_PREFIX) is not writable."
    echo "Please fix permissions or run with sudo (if using system node)."
    echo "If using nvm, ensure you are using the correct user."
    exit 1
fi

echo "Installing openchamber web interface..."

# Check if already installed and verify package integrity
PACKAGE_NAME="@openchamber/web"
PACKAGE_PATH=$(npm root -g 2>/dev/null)/$PACKAGE_NAME

if [ -d "$PACKAGE_PATH" ]; then
    echo "  Found existing installation. Checking integrity..."
    PACKAGE_JSON="$PACKAGE_PATH/package.json"
    
    # Check if package.json exists and is valid (not empty)
    if [ ! -f "$PACKAGE_JSON" ] || [ ! -s "$PACKAGE_JSON" ]; then
        echo "  Warning: Existing installation appears corrupted (empty or missing package.json)."
        echo "  Removing corrupted installation..."
        npm uninstall -g $PACKAGE_NAME 2>/dev/null || true
    fi
fi

# Use the correct package name found in the repo
echo "  Installing $PACKAGE_NAME..."
npm install -g $PACKAGE_NAME

if [ $? -ne 0 ]; then
    echo "Error: Failed to install openchamber."
    exit 1
fi

# Verify installation integrity
echo "  Verifying installation..."
PACKAGE_JSON=$(npm root -g)/$PACKAGE_NAME/package.json
if [ ! -f "$PACKAGE_JSON" ]; then
    echo "Error: Installation verification failed - package.json not found."
    exit 1
fi

if [ ! -s "$PACKAGE_JSON" ]; then
    echo "Error: Installation verification failed - package.json is empty."
    echo "  This may be an npm issue. Try running: npm uninstall -g $PACKAGE_NAME && npm install -g $PACKAGE_NAME"
    exit 1
fi

# Verify executable is available
EXEC_PATH=$(which openchamber 2>/dev/null)
if [ -z "$EXEC_PATH" ]; then
    # Try to find with hash pattern (e.g., .openchamber-Lr8RUQ7B)
    NODE_BIN_DIR=$(dirname $(which node 2>/dev/null) 2>/dev/null)
    if [ -n "$NODE_BIN_DIR" ]; then
        EXEC_PATH=$(ls -1 $NODE_BIN_DIR/.openchamber* 2>/dev/null | head -1)
    fi
fi

if [ -z "$EXEC_PATH" ]; then
    echo "Warning: openchamber executable not found in PATH after installation."
    echo "  You may need to reload your shell or check npm global bin directory."
else
    echo "  Executable found: $EXEC_PATH"
fi

echo "openchamber installation complete."
