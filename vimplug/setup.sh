#!/bin/bash

# Check for Termux environment
if [[ $HOME == *'com.termux'* ]]; then
    export IS_TERMUX=true
    echo "Running in Termux. IS_TERMUX set to true."
fi

# Neovim 설치 확인 및 설치
if ! command -v nvim &> /dev/null
then
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

# vim-plug 설치
# https://github.com/junegunn/vim-plug
VIM_PLUG_PATH="${XDG_DATA_HOME:-$HOME/.local/share}/nvim/site/autoload/plug.vim"
if [ ! -f "$VIM_PLUG_PATH" ]; then
    echo "Installing vim-plug..."
    sh -c 'curl -fLo "${XDG_DATA_HOME:-$HOME/.local/share}"/nvim/site/autoload/plug.vim --create-dirs \
           https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim'
else
    echo "vim-plug is already installed."
fi

# Silversearcher ag 설치, vim FZF 플러그인에서 활용
if ! command -v ag &> /dev/null
then
    echo "silversearcher-ag not found. Installing..."
    if [ "$IS_TERMUX" = "true" ]; then
        echo "Installing silversearcher-ag for Termux..."
        pkg install -y silversearcher-ag
    else
        echo "Installing silversearcher-ag..."
        sudo apt-get install -y silversearcher-ag
    fi
else
    echo "silversearcher-ag is already installed."
fi

# Neovim 설정 파일 복사
echo "Setting up ~/.config/nvim/init.vim"
mkdir -p ~/.config/nvim
cp vimrc ~/.config/nvim/init.vim

# Install Plugins
vim -c PlugInstall
echo "Done!"

