#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
CONFIG_SRC="$SCRIPT_DIR/config"
CONFIG_DST="$HOME/.config/opencode"

echo "Installing opencode config from $CONFIG_SRC to $CONFIG_DST..."

mkdir -p "$CONFIG_DST"

# Use rsync to copy
rsync -av "$CONFIG_SRC/" "$CONFIG_DST/"

# Setup Python virtual environment
echo "Setting up Python virtual environment..."
VENV_PATH="$CONFIG_DST/venv"

if [ ! -d "$VENV_PATH" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_PATH"
    
    echo "Installing Python dependencies..."
    source "$VENV_PATH/bin/activate"
    
    # Install dependencies if requirements.txt exists, otherwise install common packages
    if [ -f "$CONFIG_DST/requirements.txt" ]; then
        pip install -r "$CONFIG_DST/requirements.txt"
    else
        # Install common packages for opencode
        pip install openai fastapi uvicorn httpx beautifulsoup4 python-dotenv
    fi
    
    deactivate
    echo "Virtual environment setup complete."
else
    echo "Virtual environment already exists. Skipping setup."
fi

echo "Config installation complete."
