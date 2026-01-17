#!/bin/bash

# Vibe Kanban Setup Script
# This script sets up vibe-kanban with wrapper, configuration, and systemd service

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

print_status "Starting Vibe Kanban setup..."

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

# Run the installation script
if [ -f "$SCRIPT_DIR/install.sh" ]; then
    print_status "Running installation script..."
    "$SCRIPT_DIR/install.sh"
else
    print_error "Installation script not found: $SCRIPT_DIR/install.sh"
    exit 1
fi

print_status "Vibe Kanban setup completed successfully!"
echo
print_status "Next steps:"
echo "  - Configuration file: ~/.config/vibe-kanban.sh"
echo "  - Run manually: vibe-kanban"
echo "  - Service status: sudo systemctl status vibe-kanban"
echo "  - Access URL: http://$(hostname):54545 (default port)"
echo
print_status "The service is configured to start automatically on boot."