#!/bin/bash

# Vibe Kanban Service Runner - Direct execution without npm
# This runs the globally installed binary directly

# Source config file
CONFIG_FILE="$HOME/.config/vibe-kanban.sh"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
fi

# Set defaults if not configured
HOST="${VK_HOST:-0.0.0.0}"
PORT="${VK_PORT:-54545}"

# Export environment variables
export HOST="$HOST"
export PORT="$PORT"

# Direct execution of globally installed vibe-kanban binary
exec vibe-kanban "$@"