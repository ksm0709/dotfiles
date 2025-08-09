#!/bin/bash

# Termux용 D2Coding Nerd Font 설치 스크립트

# 하나라도 실패하면 즉시 중지
set -e

echo "패키지 목록을 업데이트하고 의존성(unzip, wget)을 설치합니다..."
apt-get update
apt-get install -y unzip wget

echo "~/.termux 디렉토리를 생성합니다..."
mkdir -p ~/.termux

# 임시 디렉토리 생성
TEMP_DIR=$(mktemp -d)

echo "D2Coding Nerd Font를 다운로드합니다..."
wget -O "$TEMP_DIR/D2Coding.zip" "https://github.com/ryanoasis/nerd-fonts/releases/download/v3.2.1/D2Coding.zip"

echo "폰트 압축을 해제합니다..."
unzip "$TEMP_DIR/D2Coding.zip" -d "$TEMP_DIR"
echo $TEMP_DIR

echo "폰트를 설치합니다..."
# 압축 해제된 파일 중에서 정확한 폰트 파일을 찾아 복사
FONT_FILE=$(find "$TEMP_DIR" -name "D2CodingLigatureNerdFont-Regular.ttf")
if [ -z "$FONT_FILE" ]; then
    echo "오류: D2CodingNerdFont-Regular.ttf 파일을 찾을 수 없습니다."
    exit 1
fi
cp "$FONT_FILE" ~/.termux/font.ttf

echo "임시 파일을 정리합니다..."
rm -rf "$TEMP_DIR"

echo "Termux 설정을 다시 로드하여 폰트를 적용합니다..."
termux-reload-settings

echo "D2Coding Nerd Font 설치가 완료되었습니다."
