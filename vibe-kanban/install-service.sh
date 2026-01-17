#!/bin/bash

# Vibe Kanban Service Installation Script
# Based on openchamber/install-service.sh pattern

# Arguments:
# $1: Absolute path to vibe-kanban executable
# $2: User to run the service as
# $3: Directory containing node executable (optional, for NVM/custom paths)

EXEC_PATH="$1"
SERVICE_USER="$2"
NODE_BIN_DIR="$3"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check sudo/root
if [ "$EUID" -ne 0 ]; then
  print_error "This script must be run as root (sudo)."
  exit 1
fi

# Validate arguments
if [ -z "$EXEC_PATH" ]; then
    print_error "Executable path provided is empty."
    exit 1
fi

# Ensure absolute path for EXEC_PATH
if [[ ! "$EXEC_PATH" == /* ]]; then
    echo "Debug: Resolving relative path for $EXEC_PATH"
    # 1. Try to find it in current PATH (root)
    DETECTED_PATH=$(which "$EXEC_PATH" 2>/dev/null)
    
    # 2. Try to find it in service user's PATH (with NVM support)
    if [ -z "$DETECTED_PATH" ] && [ -n "$SERVICE_USER" ] && [ "$SERVICE_USER" != "root" ]; then
        # Try a full login shell first
        DETECTED_PATH=$(sudo -u "$SERVICE_USER" -i which "$EXEC_PATH" 2>/dev/null)
        
        # If that fails, try to explicitly source NVM
        if [ -z "$DETECTED_PATH" ]; then
            USER_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
            DETECTED_PATH=$(sudo -u "$SERVICE_USER" bash -c "export NVM_DIR=\"$USER_HOME/.nvm\"; [ -s \"\$NVM_DIR/nvm.sh\" ] && . \"\$NVM_DIR/nvm.sh\"; which \"$EXEC_PATH\"" 2>/dev/null)
        fi
    fi

    # 3. Try npm global prefix for the user (with NVM support)
    if [ -z "$DETECTED_PATH" ] && [ -n "$SERVICE_USER" ]; then
        USER_NPM_PREFIX=$(sudo -u "$SERVICE_USER" bash -c "export NVM_DIR=\"\$(getent passwd $SERVICE_USER | cut -d: -f6)/.nvm\"; [ -s \"\$NVM_DIR/nvm.sh\" ] && . \"\$NVM_DIR/nvm.sh\"; npm config get prefix" 2>/dev/null)
        if [ -n "$USER_NPM_PREFIX" ] && [ -f "$USER_NPM_PREFIX/bin/$EXEC_PATH" ]; then
            DETECTED_PATH="$USER_NPM_PREFIX/bin/$EXEC_PATH"
        fi
    fi

    # 4. Broad search in NVM and local bin as fallback
    if [ -z "$DETECTED_PATH" ] && [ -n "$SERVICE_USER" ]; then
        USER_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
        echo "Debug: Searching common paths for $SERVICE_USER..."
        # Search specifically for the binary in common user-writable bin folders
        DETECTED_PATH=$(find "$USER_HOME/.nvm/versions/node" "$USER_HOME/.local/bin" "$USER_HOME/bin" -name "$EXEC_PATH" -type f -executable -print -quit 2>/dev/null | head -n 1)
    fi
    
    if [ -n "$DETECTED_PATH" ]; then
        EXEC_PATH="$DETECTED_PATH"
    else
        echo "Debug: No specific path found, falling back to realpath"
        EXEC_PATH=$(realpath "$EXEC_PATH")
    fi
fi

if [ ! -f "$EXEC_PATH" ]; then
    print_error "Executable not found at $EXEC_PATH"
    exit 1
fi

if [ -z "$SERVICE_USER" ]; then
    SERVICE_USER=${SUDO_USER}
    if [ -z "$SERVICE_USER" ]; then
         print_error "No service user specified and SUDO_USER not found."
         print_error "Usage: sudo ./install-service.sh <path_to_executable> <username> [node_bin_dir]"
         exit 1
    fi
fi

# Detect Node bin directory if not provided
if [ -z "$NODE_BIN_DIR" ]; then
    NODE_PATH=$(which node 2>/dev/null)
    
    # If not found in root path, try service user's path (with NVM support)
    if [ -z "$NODE_PATH" ] && [ -n "$SERVICE_USER" ] && [ "$SERVICE_USER" != "root" ]; then
        NODE_PATH=$(sudo -u "$SERVICE_USER" bash -c "export NVM_DIR=\"\$(getent passwd $SERVICE_USER | cut -d: -f6)/.nvm\"; [ -s \"\$NVM_DIR/nvm.sh\" ] && . \"\$NVM_DIR/nvm.sh\"; which node" 2>/dev/null)
    fi

    # Fallback to NVM detection
    if [ -z "$NODE_PATH" ] && [ -n "$SERVICE_USER" ]; then
        USER_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
        NODE_PATH=$(find "$USER_HOME/.nvm/versions/node" -name "node" -type f -executable -print -quit 2>/dev/null | sort -V | tail -n 1)
    fi

    if [ -n "$NODE_PATH" ]; then
        NODE_BIN_DIR=$(dirname "$NODE_PATH")
    fi
fi

if [ "$SERVICE_USER" == "root" ]; then
    print_warning "Running service as root is insecure. Please provide a non-root user."
fi

SERVICE_FILE="/etc/systemd/system/vibe-kanban.service"

print_status "Configuring systemd service..."
echo "  Service File: $SERVICE_FILE"
echo "  User        : $SERVICE_USER"
echo "  Exec        : $EXEC_PATH"
if [ -n "$NODE_BIN_DIR" ]; then
    echo "  Node Bin    : $NODE_BIN_DIR"
fi

# Use direct runner to avoid any wrapper conflicts
# Copy the runner to user's local bin
cp "$(dirname "$0")/vk-runner.sh" "/home/$SERVICE_USER/.local/bin/vk-runner"
chmod +x "/home/$SERVICE_USER/.local/bin/vk-runner"

# Construct Environment line for PATH if node bin is provided
ENV_PATH_LINE=""
if [ -n "$NODE_BIN_DIR" ]; then
    ENV_PATH_LINE="Environment=PATH=$NODE_BIN_DIR:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
fi

cat <<EOF > $SERVICE_FILE
[Unit]
Description=Vibe Kanban Service
After=network.target

[Service]
Type=simple
User=$SERVICE_USER
Environment=HOME=/home/$SERVICE_USER
$ENV_PATH_LINE
ExecStart=/home/$SERVICE_USER/.local/bin/vk-runner
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

print_status "Reloading systemd daemon..."
systemctl daemon-reload

print_status "Enabling vibe-kanban service..."
systemctl enable vibe-kanban

print_status "Starting vibe-kanban service..."
systemctl start vibe-kanban

echo
print_status "Status of vibe-kanban service:"
systemctl status vibe-kanban --no-pager