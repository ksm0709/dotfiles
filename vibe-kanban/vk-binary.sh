#!/bin/bash

# Vibe Kanban Service Runner - Using direct binary execution
# This script avoids npm exec entirely to prevent any wrapper conflicts

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

# Try to find the actual vibe-kanban binary in .vibe-kanban directory
VK_BIN_DIR="$HOME/.vibe-kanban/bin"
if [ -d "$VK_BIN_DIR" ]; then
    # Find the latest version directory
    LATEST_VERSION=$(ls -1 "$VK_BIN_DIR" 2>/dev/null | grep -E "^[0-9]+\." | sort -V | tail -n 1)
    if [ -n "$LATEST_VERSION" ]; then
        PLATFORM_DIR="$VK_BIN_DIR/$LATEST_VERSION"
        if [ -d "$PLATFORM_DIR" ]; then
            BINARY_PATH=$(find "$PLATFORM_DIR" -name "vibe-kanban" -type f -executable 2>/dev/null | head -n 1)
            if [ -n "$BINARY_PATH" ]; then
                echo "Found vibe-kanban binary: $BINARY_PATH"
                exec "$BINARY_PATH" "$@"
            fi
        fi
    fi
fi

# Fallback to npm if no binary found
echo "No binary found, falling back to npm..."
exec npx vibe-kanban "$@"