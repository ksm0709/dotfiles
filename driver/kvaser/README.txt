tar -xvzf linuxcan.tar.gz
sudo apt-get install build-essential
sudo apt-get install linux-headers-`uname -r`

in linuxcan folder

make
sudo make install
