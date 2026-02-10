#!/bin/bash

# Arguments:
# $1: Absolute path to openchamber executable
# $2: User to run the service as
# $3: Directory containing node executable (optional, for NVM/custom paths)

EXEC_PATH="$1"
SERVICE_USER="$2"
NODE_BIN_DIR="$3"

# Check sudo/root
if [ "$EUID" -ne 0 ]; then
  echo "Error: This script must be run as root (sudo)."
  exit 1
fi

# Validate arguments
if [ -z "$EXEC_PATH" ]; then
    echo "Error: Executable path provided is empty."
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

    # 5. Search for hash-suffixed executable (e.g., .openchamber-Lr8RUQ7B)
    # This handles npm's behavior of creating hashed binaries for some packages
    if [ -z "$DETECTED_PATH" ] && [ -n "$SERVICE_USER" ]; then
        USER_HOME=$(getent passwd "$SERVICE_USER" | cut -d: -f6)
        BASE_NAME=$(basename "$EXEC_PATH")
        
        # Try to find the executable with hash pattern
        for DIR in "$USER_HOME/.nvm/versions/node"/*/bin "$USER_HOME/.local/bin" "$USER_HOME/bin"; do
            if [ -d "$DIR" ]; then
                # Look for pattern: .basename-XXXXXXX or basename-XXXXXXX
                HASHED_EXEC=$(ls -1 "$DIR"/.$BASE_NAME-* 2>/dev/null | head -1)
                if [ -z "$HASHED_EXEC" ]; then
                    HASHED_EXEC=$(ls -1 "$DIR"/$BASE_NAME-* 2>/dev/null | head -1)
                fi
                if [ -n "$HASHED_EXEC" ] && [ -x "$HASHED_EXEC" ]; then
                    echo "Debug: Found hash-suffixed executable: $HASHED_EXEC"
                    DETECTED_PATH="$HASHED_EXEC"
                    break
                fi
            fi
        done
    fi
    
    if [ -n "$DETECTED_PATH" ]; then
        EXEC_PATH="$DETECTED_PATH"
    else
        echo "Debug: No specific path found, falling back to realpath"
        EXEC_PATH=$(realpath "$EXEC_PATH")
    fi
fi

if [ ! -f "$EXEC_PATH" ]; then
    echo "Error: Executable not found at $EXEC_PATH"
    exit 1
fi

# Verify the executable is actually executable
if [ ! -x "$EXEC_PATH" ]; then
    echo "Error: File at $EXEC_PATH is not executable"
    exit 1
fi

# Verify the executable is a valid script/binary
if ! file "$EXEC_PATH" | grep -qE '(script|executable|text)'; then
    echo "Warning: $EXEC_PATH may not be a valid executable (file type: $(file -b "$EXEC_PATH"))"
fi

if [ -z "$SERVICE_USER" ]; then
    SERVICE_USER=${SUDO_USER}
    if [ -z "$SERVICE_USER" ]; then
         echo "Error: No service user specified and SUDO_USER not found."
         echo "Usage: sudo ./install-service.sh <path_to_executable> <username> [node_bin_dir]"
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
    echo "Warning: Running service as root is insecure. Please provide a non-root user."
fi

SERVICE_FILE="/etc/systemd/system/openchamber.service"

echo "Configuring systemd service..."
echo "  Service File: $SERVICE_FILE"
echo "  User        : $SERVICE_USER"
echo "  Exec        : $EXEC_PATH"
echo "  Port        : 8888"
if [ -n "$NODE_BIN_DIR" ]; then
    echo "  Node Bin    : $NODE_BIN_DIR"
fi

# Construct Environment line for PATH
# Always include NODE_BIN_DIR if available (critical for NVM users)
# Include standard system paths as fallback
if [ -n "$NODE_BIN_DIR" ]; then
    ENV_PATH_LINE="Environment=PATH=$NODE_BIN_DIR:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
else
    # Fallback to standard paths if NODE_BIN_DIR couldn't be detected
    ENV_PATH_LINE="Environment=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
fi

cat <<EOF > $SERVICE_FILE
[Unit]
Description=OpenChamber Service
After=network.target

[Service]
Type=simple
User=$SERVICE_USER
Environment=HOME=/home/$SERVICE_USER
$ENV_PATH_LINE
ExecStart=$EXEC_PATH --port 8888
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling openchamber service..."
systemctl enable openchamber

echo "Starting openchamber service..."
systemctl start openchamber

echo "Status of openchamber service:"
systemctl status openchamber --no-pager
