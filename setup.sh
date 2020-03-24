#!/bin/bash

sudo apt-get install tmux -qq
sudo apt-get install vim -qq
cp tmux.conf ../.tmux.conf
cp vimrc ../.vimrc

mkdir ${HOME}/.vim
mkdir ${HOME}/.vim/bundle

git clone https://github.com/gmarik/Vundle.vim.git ~/.vim/bundle/Vundle.vim

vim -c PluginInstall

