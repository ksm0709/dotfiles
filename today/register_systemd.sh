#!/bin/bash

# Script to register make_today.sh as a systemd service and add an alias

# Define variables
SCRIPT_PATH=$(readlink -f "$(dirname "$0")/make_today.sh") # Absolute path to make_today.sh
SERVICE_FILE="/etc/systemd/system/workdir.service"
BASHRC="$HOME/.bashrc"
ALIAS_COMMAND="alias today='cd $HOME/today'"
USER=$(whoami)

# Check if make_today.sh exists and is executable
if [ ! -x "$SCRIPT_PATH" ]; then
	echo "Error: make_today.sh does not exist or is not executable.  Run 'chmod +x make_today.sh'."
	exit 1
fi

# Create the systemd service file
cat <<EOF | sudo tee "$SERVICE_FILE"
[Unit]
Description=Create work directory and symlink on boot
After=network.target

[Service]
Type=oneshot
ExecStart=$(which bash) "$SCRIPT_PATH"
User=$USER
#WorkingDirectory=$HOME

[Install]
WantedBy=multi-user.target
EOF

# Fix permission after the creation of the file
sudo chmod 644 "$SERVICE_FILE"

# Enable and start the service
sudo systemctl enable "workdir.service"
sudo systemctl start "workdir.service"

# Check the service status
sudo systemctl status "workdir.service"

# Add the alias to .bashrc
if grep -q "$ALIAS_COMMAND" "$BASHRC"; then
	echo "Alias 'today' already exists in $BASHRC"
else
	echo "Adding alias 'today' to $BASHRC"
	echo "$ALIAS_COMMAND" >>"$BASHRC"
	echo "Remember to source $BASHRC or open a new terminal to use the alias."
fi

echo "Systemd service 'workdir.service' created, enabled, started, and status checked."

#Make the register script executable as well
chmod +x "$0"
