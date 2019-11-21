#!/bin/bash
sudo apt-get install software-properties-common

sudo add-apt-repository ppa:neovim-ppa/stable
sudo apt-get update
sudo apt-get install neovim

sudo apt-get install python-dev python-pip python3-dev python3-pip

sudo update-alternatives --install /usr/bin/vi vi /usr/bin/nvim 60
sudo update-alternatives --config vi
sudo update-alternatives --install /usr/bin/vim vim /usr/bin/nvim 60
sudo update-alternatives --config vim
sudo update-alternatives --install /usr/bin/editor editor /usr/bin/nvim 60
sudo update-alternatives --config editor

# spacevim
curl -sLf https://spacevim.org/install.sh | bash
mkdir $HOME/.SpaceVim.d/autoload

cp init.toml $HOME/.SpaceVim.d
cp config.vim $HOME/.SpaceVim.d/autoload

# gtags
tar xvf global-6.6.3.tar.gz
cd global-6.6.3

./configure --with-exuberant-ctags=/usr/bin/ctags
make
sudo make install

cp gtags.conf ~/.globalrc
echo export GTAGSLABEL=pygments >> .profile

cd ..
