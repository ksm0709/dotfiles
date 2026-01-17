#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

echo "Setting up opencode..."

# Make scripts executable just in case
chmod +x "$SCRIPT_DIR/install.sh"
chmod +x "$SCRIPT_DIR/install-config.sh"
chmod +x "$SCRIPT_DIR/backup-config.sh"

# Install opencode
"$SCRIPT_DIR/install.sh"

# Install config and setup venv
"$SCRIPT_DIR/install-config.sh"

echo "Setup complete."
echo "Note: Python virtual environment has been set up in $HOME/.config/opencode/venv"
