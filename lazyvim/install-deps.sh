#!/bin/bash

# Apt deps
pkg_list=( \
  clang \
  make \
  wget \
  luarocks \
  fzf \
  imagemagick \
  lazygit \
  fd-find \
  chromium \
)

#if its termux, use pkg
if [ -f /data/data/com.termux/files/usr/bin/pkg ]; then
  # ignore unexisting packages
  for pkg in "${pkg_list[@]}"; do
    if ! pkg info "$pkg" > /dev/null 2>&1; then
      echo "Package $pkg not found. Skipping..."
      continue
    fi
    pkg install "$pkg"
  done
  exit 0
fi
sudo apt-get install "${pkg_list[@]}"
