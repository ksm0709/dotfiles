#!/bin/bash

USE_STARTER_PACK=${1:-1}

if ${USE_STARTER_PACK}; then
  # Clone lazyvim starter pack
  # Ref: https://www.lazyvim.org/installationon
  ./backup-nvim-config.sh
  git clone https://github.com/LazyVim/starter ~/.config/nvim
  rm -rf ~/.config/nvim/.git
else
  echo "Setting up custom LazyVim configs"
  mkdir -p ~/.config/nvim/lua
  cp -r ./nvim/* ~/.config/nvim/
fi
