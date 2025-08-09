#!/bin/bash

# Clone lazyvim starter pack
# Ref: https://www.lazyvim.org/installationon
./backup-nvim-config.sh
echo "Clone lazyvim starter pack"
git clone https://github.com/LazyVim/starter ~/.config/nvim
rm -rf ~/.config/nvim/.git
