#!/bin/bash

echo "Setting up custom LazyVim configs"
mkdir -p ~/.config/nvim/lua
if [[ "$PREFIX" == *"com.termux"* ]] || [ -d "/data/data/com.termux" ]; then
	echo "Termux detected, installing nvim-termux configs"
	cp -r ./nvim-termux/* ~/.config/nvim/
else
	echo "Installing default nvim configs"
	cp -r ./nvim/* ~/.config/nvim/
fi
