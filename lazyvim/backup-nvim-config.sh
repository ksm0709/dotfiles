#!/bin/bash
mkdir -p ~/.config/nvim
rm -rf ~/.config/nvim.bak
mv ~/.config/nvim{,.bak}
rm -rf ~/.local/share/nvim.bak
rm -rf ~/.local/state/nvim.bak
rm -rf ~/.cache/nvim.bak
mv ~/.local/share/nvim{,.bak}
mv ~/.local/state/nvim{,.bak}
mv ~/.cache/nvim{,.bak}
