#!/bin/bash

echo -e "- Install Tmux"
sudo apt-get install tmux -qq

echo -e "- Copying config file"
cp tmux.conf ../.tmux.conf
