#!/bin/bash

# Colors for output
RED='\033[1;31m'
GREEN='\033[1;32m'
ORANGE='\033[1;33m'
BLUE='\033[1;34m'
NC='\033[0m'

echo -e "${BLUE}>> Installing Tailscale...${NC}"

# Check if curl is available
if ! command -v curl &> /dev/null; then
    echo -e "${RED}✗ Error: curl is not installed. Please install curl first.${NC}"
    exit 1
fi

# Check if we have network connectivity
if ! curl -s --connect-timeout 5 https://tailscale.com > /dev/null; then
    echo -e "${RED}✗ Error: Unable to connect to tailscale.com. Please check your internet connection.${NC}"
    exit 1
fi

# Install Tailscale using official installer
echo -e "${ORANGE}Downloading and installing Tailscale...${NC}"
if ! curl -fsSL https://tailscale.com/install.sh | sh; then
    echo -e "${RED}✗ Tailscale installation failed.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Tailscale installed successfully.${NC}"

# Start Tailscale with SSH enabled
echo -e "${BLUE}>> Starting Tailscale with SSH enabled...${NC}"
if ! sudo tailscale up --ssh; then
    echo -e "${RED}✗ Failed to start Tailscale with SSH enabled.${NC}"
    echo -e "${ORANGE}Note: You may need to authenticate by visiting the URL that will be displayed.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Tailscale started with SSH enabled.${NC}"

# Display status information
echo -e "${BLUE}>> Tailscale Status:${NC}"
if command -v tailscale status &> /dev/null; then
    tailscale status
else
    echo -e "${ORANGE}Unable to display status. You may need to authenticate first.${NC}"
fi

echo -e "${GREEN}✓ Tailscale installation and configuration completed successfully!${NC}"
echo -e "${ORANGE}Note: If this is your first time, visit the authentication URL to connect to your account.${NC}"