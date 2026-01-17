#!/bin/bash

# Vibe Kanban Service Runner - Direct Execution without Wrapper Conflicts
# This script bypasses any potential wrapper conflicts entirely

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

# Clear any potential PATH issues that might cause wrapper conflicts
# Run vibe-kanban directly by calling npm with explicit execution
exec npm exec vibe-kanban "$@"