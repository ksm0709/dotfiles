#!/bin/bash

# Vibe Kanban Wrapper
# Reads configuration from ~/.config/vibe-kanban.sh and runs vibe-kanban

# Source config file
CONFIG_FILE="$HOME/.config/vibe-kanban.sh"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
fi

# Set defaults if not configured
HOST="${VK_HOST:-127.0.0.1}"
PORT="${VK_PORT:-54747}"

# Export environment variables
export HOST="$HOST"
export PORT="$PORT"

# Run npx vibe-kanban directly
exec npx vibe-kanban "$@"