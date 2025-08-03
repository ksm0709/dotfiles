#!/bin/bash

# Apt deps
sudo apt-get install -y \
	wget \
	luarocks \
	fzf \
	imagemagick \
	lazygit

# Tecthnic - for snacks.image to render latex
# Check latest version if needed: https://tectonic-typesetting.github.io/latest.html
wget https://github.com/tectonic-typesetting/tectonic/releases/download/tectonic%400.15.0/tectonic-0.15.0-aarch64-unknown-linux-musl.tar.gz -o tectonic.tar.gz
tar -xvf tectonic.tar.gz
mv tectonic /usr/bin/
rm tecthonic.tar.gz

# mermaid-cli for snacks.image to render mermaid
npm install -g @mermaid-js/mermaid-cli
