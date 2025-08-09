#!/bin/bash

# Apt deps
pkg_list = (
  clang
  make
  wget
  luarocks
  fzf
  imagemagick
  lazygit
  fd-find
  chromium
)

#if its termux, use pkg
if [ -f /data/data/com.termux/files/usr/bin/pkg ]; then
  pkg install "${pkg_list[@]}"
  exit 0

sudo apt-get install "${pkg_list[@]}"
