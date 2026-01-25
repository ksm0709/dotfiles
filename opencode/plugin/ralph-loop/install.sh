#!/bin/bash

# Ralph Loop Plugin Installer for OpenCode

set -e

PLUGIN_NAME="ralph-loop"
TARGET_DIR="$HOME/.config/opencode/plugin/$PLUGIN_NAME"
SOURCE_DIR=$(dirname "$(readlink -f "$0")")

echo "📦 Installing $PLUGIN_NAME to $TARGET_DIR..."

# 1. 디렉토리 생성
mkdir -p "$TARGET_DIR"

# 2. 소스 복사 (node_modules 제외)
cp -r "$SOURCE_DIR/src" "$TARGET_DIR/"
cp "$SOURCE_DIR/package.json" "$TARGET_DIR/"
cp "$SOURCE_DIR/tsconfig.json" "$TARGET_DIR/"
[ -f "$SOURCE_DIR/eslint.config.mjs" ] && cp "$SOURCE_DIR/eslint.config.mjs" "$TARGET_DIR/"
[ -f "$SOURCE_DIR/.prettierrc" ] && cp "$SOURCE_DIR/.prettierrc" "$TARGET_DIR/"

# 3. 의존성 설치
echo "📥 Installing dependencies..."
cd "$TARGET_DIR"
npm install --production

echo ""
echo "✅ Installation complete!"
echo "🚀 To enable the plugin, add the following to your ~/.config/opencode/opencode.json:"
echo ""
echo "{"
echo "  \"plugin\": ["
echo "    \"./plugin/$PLUGIN_NAME/src/index.ts\""
echo "  ],"
echo "  \"$PLUGIN_NAME\": {"
echo "    \"promiseWord\": \"DONE\","
echo "    \"maxRetries\": 5"
echo "  }"
echo "}"
