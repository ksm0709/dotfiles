#!/bin/bash

# Vibe Kanban Service Script
# This script runs vibe-kanban directly, reading config from ~/.config/vibe-kanban.sh

# Source config file
CONFIG_FILE="$HOME/.config/vibe-kanban.sh"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
fi

# Set defaults if not configured
HOST="${VK_HOST:-127.0.0.1}"
PORT="${VK_PORT:-8888}"

# Export environment variables and run vibe-kanban
export HOST="$HOST"
export PORT="$PORT"

# Run vibe-kanban using npx directly
exec npx vibe-kanban "$@"