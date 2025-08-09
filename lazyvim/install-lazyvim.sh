#!/bin/bash

# Clone lazyvim starter pack
# Ref: https://www.lazyvim.org/installationon
./backup-nvim-config.sh
echo "Clone lazyvim starter pack"
git clone https://github.com/LazyVim/starter ~/.config/nvim
rm -rf ~/.config/nvim/.git

#echo "Setting up custom LazyVim configs"
#mkdir -p ~/.config/nvim/lua
#cp -r ./nvim/* ~/.config/nvim/
