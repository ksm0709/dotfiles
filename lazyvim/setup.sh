#!/bin/bash

# Neovim
./install-nvim.sh

# Install lazyvim
#  1 - use starter pack
#  0 - use custom lazyvim configs
./install-lazyvim.sh 1

# Install dependent packages & binaries
./install-deps.sh
