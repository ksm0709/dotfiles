# 우분투 세팅

## 기본세팅

```
sudo apt-get install tmux -qq
sudo apt-get install vim -qq
cp tmux.conf ../.tmux.conf
cp vimrc ../.vimrc
git clone https://github.com/gmarik/Vundle.vim.git ~/.vim/bundle/Vundle.vim
```

- vim 플러그인 설치
```
vim
:PluginInstall
````
==========================

## ROS설치

```
cd ros

./ros_install.sh <workspace name> <ros distro>
```

==========================

## VS Code Sync

- Access Token : **1ba0a4fe28c3811da78c6ad38e1fc411f5c59a4d**
