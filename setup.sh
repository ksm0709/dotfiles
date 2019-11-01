#!/bin/bash

sudo apt-get install tmux -qq
sudo apt-get install vim -qq
cp tmux.conf ../.tmux.conf

# spacevim style
curl -sLf https://spacevim.org/install.sh | bash
cp init.toml ../.SpaceVim.d/init.toml
vim

# old vim style
#cp vimrc ../.vimrc
#git clone https://github.com/gmarik/Vundle.vim.git ~/.vim/bundle/Vundle.vim
#vim -c PluginInstall

