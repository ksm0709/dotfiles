#!/bin/bash

# Neovim 설치 확인 및 설치
if ! command -v nvim &>/dev/null; then
	echo "Neovim not found. Installing..."
	sudo apt-get install software-properties-common

	sudo add-apt-repository ppa:neovim-ppa/stable
	sudo apt-get update
	sudo apt-get install neovim

	sudo apt-get install python-dev python-pip python3-dev python3-pip

	# Neovim을 기본 편집기로 설정
	sudo update-alternatives --install /usr/bin/vi vi /usr/bin/nvim 60
	sudo update-alternatives --install /usr/bin/vim vim /usr/bin/nvim 60
	sudo update-alternatives --install /usr/bin/editor editor /usr/bin/nvim 60
else
	echo "Neovim is already installed."
fi
mkdir -p ~/.config/nvim

### Install LazyVimm
### Ref: https://www.lazyvim.org/installationon
# required
mv ~/.config/nvim{,.bak}
# optional but recommended
mv ~/.local/share/nvim{,.bak}
mv ~/.local/state/nvim{,.bak}
mv ~/.cache/nvim{,.bak}

git clone https://github.com/LazyVim/starter ~/.config/nvim
rm -rf ~/.config/nvim/.git
###

# Neovim 설정 파일 복사
echo "Setting up custom LazyVim configs"
mkdir -p ~/.config/nvim/lua
cp -r ./lua ~/.config/nvim/
