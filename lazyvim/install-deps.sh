#!/bin/bash

# Apt deps
sudo apt-get install -y \
	wget \
	luarocks \
	fzf \
	imagemagick \
	lazygit \
	fd-find

# mermaid-cli for snacks.image to render mermaid
npm install -g @mermaid-js/mermaid-cli
