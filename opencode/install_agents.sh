#!/bin/bash

# 설정
SOURCE_DIR="config/agent"
TARGET_DIR="$HOME/.config/opencode/agent"

# 소스 디렉토리 확인
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Error: Source directory '$SOURCE_DIR' not found."
    exit 1
fi

# 대상 디렉토리 생성
if [ ! -d "$TARGET_DIR" ]; then
    echo "Creating target directory: $TARGET_DIR"
    mkdir -p "$TARGET_DIR"
fi

# 파일 복사 (업데이트)
echo "Installing/Updating agent configurations..."
cp -v "$SOURCE_DIR"/*.md "$TARGET_DIR/"

if [ $? -eq 0 ]; then
    echo "Successfully installed agent configurations to $TARGET_DIR"
else
    echo "Error: Failed to copy files."
    exit 1
fi
