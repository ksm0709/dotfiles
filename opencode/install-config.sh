#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
CONFIG_SRC="$SCRIPT_DIR/config"
CONFIG_DST="$HOME/.config/opencode"

echo "Installing opencode config from $CONFIG_SRC to $CONFIG_DST..."

mkdir -p "$CONFIG_DST"

# Use rsync to copy
rsync -av --exclude 'venv' "$CONFIG_SRC/" "$CONFIG_DST/"

# Install omo-profile if the command is not available
_OMO_PROFILE_SCRIPT="$CONFIG_DST/profiles/omo-profile.sh"

if ! type omo-profile &>/dev/null; then
	echo "Installing omo-profile locally..."

	_SHELL_RC=""
	case "$(basename "${SHELL:-bash}")" in
		zsh) _SHELL_RC="$HOME/.zshrc" ;;
		*) _SHELL_RC="$HOME/.bashrc" ;;
	esac

	if [ -n "$_SHELL_RC" ] && ! grep -q "omo-profile.sh" "$_SHELL_RC" 2>/dev/null; then
		printf '\n# oh-my-opencode profile switcher\nsource "%s"\n' "$_OMO_PROFILE_SCRIPT" >>"$_SHELL_RC"
		echo "omo-profile added to $_SHELL_RC"
	else
		echo "omo-profile already configured in $_SHELL_RC"
	fi
fi

# Source omo-profile for immediate use in this script
# shellcheck source=/dev/null
source "$_OMO_PROFILE_SCRIPT"

# Create oh-my-opencode.json symlink via omo-profile (default profile)
if [ ! -e "$CONFIG_DST/oh-my-opencode.json" ]; then
	echo "Setting up default profile link..."
	omo-profile use default
fi

# Setup Python virtual environment
echo "Setting up Python virtual environment..."
VENV_PATH="$CONFIG_DST/venv"

if [ ! -d "$VENV_PATH" ]; then
	echo "Creating virtual environment..."
	python3 -m venv "$VENV_PATH"

	echo "Installing Python dependencies..."
	source "$VENV_PATH/bin/activate"

	# Install dependencies if requirements.txt exists, otherwise install common packages
	if [ -f "$CONFIG_DST/requirements.txt" ]; then
		pip install -r "$CONFIG_DST/requirements.txt"
	else
		# Install common packages for opencode
		pip install openai fastapi uvicorn httpx beautifulsoup4 python-dotenv
	fi

	deactivate
	echo "Virtual environment setup complete."
else
	echo "Virtual environment already exists. Skipping setup."
fi

echo "Config installation complete."
