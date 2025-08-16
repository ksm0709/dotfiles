#!/bin/bash

echo "Copying nvim configs from ~/.config/nvim to this repo"

if [[ "$PREFIX" == *"com.termux"* ]] || [ -d "/data/data/com.termux" ]; then
	echo "Termux detected, copying to lazyvim/nvim-termux/"
	mkdir -p ./nvim-termux
	cp -r ~/.config/nvim/* ./nvim-termux/
else
	echo "Default system detected, copying to lazyvim/nvim/"
	mkdir -p ./nvim
	cp -r ~/.config/nvim/* ./nvim/
fi

echo "Done!"
