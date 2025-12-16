#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
CONFIG_SRC="$HOME/.config/opencode"
CONFIG_DST="$SCRIPT_DIR/config"

if [ ! -d "$CONFIG_SRC" ]; then
    echo "Error: Source config directory $CONFIG_SRC does not exist."
    exit 1
fi

echo "Backing up opencode config from $CONFIG_SRC to $CONFIG_DST..."

mkdir -p "$CONFIG_DST"

# Use rsync to copy and exclude node_modules and .git
rsync -av --delete --exclude 'node_modules' --exclude '.git' "$CONFIG_SRC/" "$CONFIG_DST/"

echo "Backup complete."
