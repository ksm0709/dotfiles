#!/bin/bash

# ByteRover Installation Script
# Verifies dependencies and installs byterover-cli

set -e

echo "🔧 ByteRover Installation Script"
echo "================================"

# Check Node.js version
echo "📦 Checking Node.js..."
if ! command -v node &>/dev/null; then
	echo "❌ Node.js is not installed. Please install Node.js 20 or higher."
	exit 1
fi

NODE_VERSION=$(node -v | cut -d'v' -f2)
REQUIRED_VERSION="20.0.0"

if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$NODE_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
	echo "❌ Node.js version $NODE_VERSION is too old. Requires Node.js 20 or higher."
	exit 1
fi

echo "✅ Node.js $NODE_VERSION detected"

# Check npm
if ! command -v npm &>/dev/null; then
	echo "❌ npm is not installed."
	exit 1
fi

echo "✅ npm detected"

# Check for libsecret on Linux
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
	echo "🔐 Checking for libsecret (Linux credential storage)..."
	if ! dpkg -l | grep -q libsecret-1-dev; then
		echo "⚠️  libsecret-1-dev not found. Installing..."
		sudo apt update && sudo apt install -y libsecret-1-dev
	else
		echo "✅ libsecret-1-dev is installed"
	fi
fi

# Install or update byterover-cli
echo "🚀 Installing byterover-cli..."
if command -v brv &>/dev/null; then
	echo "📦 Updating byterover-cli..."
	npm update -g byterover-cli
else
	echo "📦 Installing byterover-cli globally..."
	npm install -g byterover-cli
fi

echo "✅ Installation complete!"
echo ""
echo "In your project, Start with: brv"
