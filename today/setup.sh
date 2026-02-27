#!/bin/bash

# Define checks
SERVICE_NAME="workdir.service"
TODAY_LINK="$HOME/today"
BASHRC="$HOME/.bashrc"

echo "Checking current installation status..."

# 1. Check if service is enabled
if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
	echo "[OK] Service '$SERVICE_NAME' is enabled."
	SERVICE_ENABLED=true
else
	echo "[MISSING] Service '$SERVICE_NAME' is not enabled or not found."
	SERVICE_ENABLED=false
fi

# 2. Check if symlink exists
if [ -L "$TODAY_LINK" ]; then
	echo "[OK] Symlink '$TODAY_LINK' exists."
	LINK_EXISTS=true
else
	echo "[MISSING] Symlink '$TODAY_LINK' does not exist."
	LINK_EXISTS=false
fi

# 3. Check for alias
if grep -q "alias today=" "$BASHRC"; then
	echo "[OK] Alias 'today' found in $BASHRC."
	ALIAS_EXISTS=true
else
	echo "[MISSING] Alias 'today' not found in $BASHRC."
	ALIAS_EXISTS=false
fi

# Decision
if [ "$SERVICE_ENABLED" = true ] && [ "$LINK_EXISTS" = true ] && [ "$ALIAS_EXISTS" = true ]; then
	echo "Everything looks good. No changes needed."
	exit 0
fi

echo "Installation incomplete or missing components. Proceeding with setup..."

# Ensure the scripts are executable
echo "Setting executable permissions for make_today.sh and register_systemd.sh..."
chmod +x make_today.sh
chmod +x register_systemd.sh

# Run the systemd registration script
echo "Running register_systemd.sh..."
./register_systemd.sh

echo "Done!"
