#!/bin/bash
# This script downloads and installs the D2Coding Nerd Font for the current user.

# Variables
FONT_NAME="D2Coding"
FONT_URL="https://github.com/ryanoasis/nerd-fonts/releases/latest/download/D2Coding.zip"
FONT_DIR="${HOME}/.local/share/fonts"
TEMP_FILE="/tmp/${FONT_NAME}.zip"

# Check for required commands
if ! command -v wget &> /dev/null; then
    echo "Error: wget is not installed. Please install it first."
    exit 1
fi

if ! command -v unzip &> /dev/null; then
    echo "Error: unzip is not installed. Please install it first."
    exit 1
fi

# Create font directory if it doesn't exist
echo "Creating font directory at ${FONT_DIR}..."
mkdir -p "${FONT_DIR}"

# Download the font
echo "Downloading ${FONT_NAME} Nerd Font..."
wget -q --show-progress -O "${TEMP_FILE}" "${FONT_URL}"
if [ $? -ne 0 ]; then
    echo "Error: Font download failed."
    exit 1
fi

# Unzip the font
echo "Installing font to ${FONT_DIR}..."
unzip -o "${TEMP_FILE}" -d "${FONT_DIR}"
if [ $? -ne 0 ]; then
    echo "Error: Font extraction failed."
    exit 1
fi

# Clean up
echo "Cleaning up temporary files..."
rm "${TEMP_FILE}"

# Update font cache
echo "Updating font cache..."
fc-cache -fv

echo "${FONT_NAME} Nerd Font installed successfully!"
