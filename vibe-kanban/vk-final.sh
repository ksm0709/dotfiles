#!/bin/bash

# Vibe Kanban Final Service Solution
# This completely avoids the infinite loop by NOT using any wrapper names

# Source config file  
CONFIG_FILE="$HOME/.config/vibe-kanban.sh"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
fi

# Set defaults
HOST="${VK_HOST:-0.0.0.0}"
PORT="${VK_PORT:-54545}"

# Export to environment
export HOST="$HOST"
export PORT="$PORT"

# Direct execution without any wrapper conflicts
# This completely bypasses npm exec to avoid the internal "vibe-kanban" call issue
VIBE_DIR="$HOME/.vibe-kanban"
if [ -d "$VIBE_DIR" ]; then
    # Find the latest installed version
    LATEST_VERSION=$(ls -1 "$VIBE_DIR/bin" | grep -E "^[0-9]+\." | sort -V | tail -n 1)
    if [ -n "$LATEST_VERSION" ]; then
        BINARY="$VIBE_DIR/bin/$LATEST_VERSION/linux-x64/vibe-kanban"
        if [ -x "$BINARY" ]; then
            exec "$BINARY" "$@"
        fi
    fi
fi

# Final fallback if no binary found
echo "Error: No vibe-kanban binary found. Please run npx vibe-kanban once to install."
exit 1