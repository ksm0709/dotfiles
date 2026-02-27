#!/bin/bash

# Get today's date in YYYY-MM-DD format
TODAY=$(date +%Y-%m-%d)

# Define the work directory and the target directory for today's date
WORK_DIR="$HOME/work"
TODAY_DIR="$WORK_DIR/$TODAY"
TODAY_LINK="$HOME/today"

# Create the work directory if it doesn't exist
mkdir -p "$WORK_DIR"

# Create the directory for today's date if it doesn't exist
if [ ! -d "$TODAY_DIR" ]; then
	echo "Creating directory: $TODAY_DIR"
	mkdir "$TODAY_DIR"
else
	echo "Directory $TODAY_DIR already exists."
fi

# Create or update the symbolic link
if [ -L "$TODAY_LINK" ]; then
	LINK_TARGET=$(readlink "$TODAY_LINK")
	if [ "$LINK_TARGET" != "$TODAY_DIR" ]; then
		echo "Updating symbolic link: $TODAY_LINK -> $TODAY_DIR"
		rm "$TODAY_LINK"
		ln -s "$TODAY_DIR" "$TODAY_LINK"
	else
		echo "Symbolic link $TODAY_LINK already points to $TODAY_DIR."
	fi
else
	echo "Creating symbolic link: $TODAY_LINK -> $TODAY_DIR"
	ln -s "$TODAY_DIR" "$TODAY_LINK"
fi

echo "Script completed successfully."
