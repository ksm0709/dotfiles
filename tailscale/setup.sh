#!/bin/bash

# Colors for output
RED='\033[1;31m'
GREEN='\033[1;32m'
ORANGE='\033[1;33m'
BLUE='\033[1;34m'
NC='\033[0m'

# Change to script directory
cd "$(dirname "$0")"

echo -e "${BLUE}>> ${ORANGE} Setting up Tailscale... ${NC}"

# Make install script executable
chmod +x install.sh

# Run the installation
./install.sh

# Check if installation was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Tailscale setup completed successfully!${NC}"
else
    echo -e "${RED}✗ Tailscale setup failed. Please check the output above.${NC}"
    exit 1
fi