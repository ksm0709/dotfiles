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
# Use the correct package name found in the repo
npm install -g @openchamber/web

if [ $? -eq 0 ]; then
    echo "openchamber installation complete."
else
    echo "Failed to install openchamber."
    exit 1
fi
