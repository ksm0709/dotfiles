#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
CONFIG_SRC="$SCRIPT_DIR/config"
CONFIG_DST="$HOME/.config/opencode"

echo "Installing opencode config from $CONFIG_SRC to $CONFIG_DST..."

mkdir -p "$CONFIG_DST"

# Use rsync to copy
rsync -av "$CONFIG_SRC/" "$CONFIG_DST/"

echo "Config installation complete."
