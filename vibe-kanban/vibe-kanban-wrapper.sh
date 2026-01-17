#!/bin/bash

# Vibe Kanban Wrapper
# Reads configuration from ~/.config/vibe-kanban.yaml and runs vibe-kanban

CONFIG_FILE="$HOME/.config/vibe-kanban.yaml"

# Parse YAML configuration using Python3
if [ -f "$CONFIG_FILE" ]; then
    eval $(python3 -c "
import yaml, sys
try:
    with open('$CONFIG_FILE', 'r') as f:
        cfg = yaml.safe_load(f)
        if cfg and isinstance(cfg, dict):
            for k, v in cfg.items():
                if v is not None:
                    print(f'export VK_{k.upper()}=\"{v}\"')
except Exception:
    pass
")
fi

# Set defaults if not configured
export HOST="${VK_HOST:-0.0.0.0}"
export PORT="${VK_PORT:-54545}"

# Run the actual binary if it exists, otherwise fallback to npx
# This avoids infinite loop if the wrapper is named 'vibe-kanban'
if [ -f "/usr/local/lib/node_modules/vibe-kanban/bin/cli.js" ]; then
    exec node /usr/local/lib/node_modules/vibe-kanban/bin/cli.js "$@"
else
    exec npx vibe-kanban "$@"
fi
