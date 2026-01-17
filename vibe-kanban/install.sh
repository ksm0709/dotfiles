#!/bin/bash

# Vibe Kanban Installation Script
# Installs vibe-kanban wrapper, configuration, and systemd service

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root for service installation
if [[ $EUID -eq 0 ]]; then
    print_error "This script should not be run as root directly."
    print_error "Run it as your regular user, and it will ask for sudo when needed."
    exit 1
fi

# Check if npm is installed
if ! command -v npm &> /dev/null; then
    print_error "npm is not installed. Please install Node.js and npm first."
    exit 1
fi

print_status "npm found: $(npm --version)"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WRAPPER_SCRIPT="$SCRIPT_DIR/vibe-kanban-wrapper.sh"
CONFIG_TEMPLATE="$SCRIPT_DIR/config-template.sh"
SERVICE_INSTALL_SCRIPT="$SCRIPT_DIR/install-service.sh"

# Check if required files exist
if [ ! -f "$WRAPPER_SCRIPT" ]; then
    print_error "Wrapper script not found: $WRAPPER_SCRIPT"
    exit 1
fi

if [ ! -f "$CONFIG_TEMPLATE" ]; then
    print_error "Config template not found: $CONFIG_TEMPLATE"
    exit 1
fi

if [ ! -f "$SERVICE_INSTALL_SCRIPT" ]; then
    print_error "Service install script not found: $SERVICE_INSTALL_SCRIPT"
    exit 1
fi

# Create config directory if it doesn't exist
CONFIG_DIR="$HOME/.config"
CONFIG_FILE="$CONFIG_DIR/vibe-kanban.sh"

print_status "Setting up configuration..."
mkdir -p "$CONFIG_DIR"

# Create config file if it doesn't exist
if [ ! -f "$CONFIG_FILE" ]; then
    print_status "Creating default config file: $CONFIG_FILE"
    cp "$CONFIG_TEMPLATE" "$CONFIG_FILE"
    chmod +x "$CONFIG_FILE"
    print_status "Default configuration created with HOST=0.0.0.0 and PORT=54545"
    print_status "You can modify it at: $CONFIG_FILE"
else
    print_warning "Configuration file already exists: $CONFIG_FILE"
    print_warning "Keeping existing configuration."
fi

# Install wrapper script system-wide
WRAPPER_TARGET="/usr/local/bin/vibe-kanban"

print_status "Installing vibe-kanban globally..."
if npm install -g vibe-kanban; then
    print_status "vibe-kanban installed globally."
else
    print_error "Failed to install vibe-kanban globally."
    exit 1
fi

print_status "Installing wrapper script to $WRAPPER_TARGET..."

if sudo cp "$WRAPPER_SCRIPT" "$WRAPPER_TARGET"; then
    sudo chmod +x "$WRAPPER_TARGET"
    print_status "Wrapper script installed successfully."
else
    print_error "Failed to install wrapper script."
    exit 1
fi

# Install systemd service
print_status "Installing systemd service..."
if sudo bash "$SERVICE_INSTALL_SCRIPT" "$WRAPPER_TARGET" "$USER"; then
    print_status "Systemd service installed successfully."
else
    print_error "Failed to install systemd service."
    exit 1
fi

# Installation complete
print_status "Vibe Kanban installation complete!"
echo
print_status "Usage:"
echo "  - Run manually: vibe-kanban"
echo "  - Service status: sudo systemctl status vibe-kanban"
echo "  - Configuration file: $CONFIG_FILE"
echo
print_status "The service will start automatically on boot."
print_status "You can also start it now with: sudo systemctl start vibe-kanban"