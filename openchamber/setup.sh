#!/bin/bash

# 스크립트 디렉토리로 이동
cd "$(dirname "$0")"

echo ">> Installing openchamber..."
./install.sh

if [ $? -ne 0 ]; then
    echo "Installation failed. Aborting service setup."
    exit 1
fi

echo ">> Setting up openchamber systemd service..."
# bun이 패키지 경로와 엔트리포인트를 내부적으로 탐색합니다.
sudo ./install-service.sh "$USER"
