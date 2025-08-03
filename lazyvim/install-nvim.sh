#!/bin/bash

if command -v nvim &>/dev/null; then
  echo "Neovim is already installed."
  exit 1
fi

echo "Neovim not found. Installing..."
sudo apt-get install software-properties-common

sudo add-apt-repository ppa:neovim-ppa/unstable
sudo apt-get update
sudo apt-get install neovim

sudo apt-get install python-dev python-pip python3-dev python3-pip

# Neovim을 기본 편집기로 설정
sudo update-alternatives --install /usr/bin/vi vi /usr/bin/nvim 60
sudo update-alternatives --install /usr/bin/vim vim /usr/bin/nvim 60
sudo update-alternatives --install /usr/bin/editor editor /usr/bin/nvim 60
