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

# 3. 기본 설정 파일 생성 (없을 경우에만)
if [ ! -f "$TARGET_DIR/ralph-loop.json" ]; then
    echo "📝 Creating default ralph-loop.json..."
    cat <<EOF > "$TARGET_DIR/ralph-loop.json"
{
  "promiseWord": "DONE",
  "maxRetries": 5,
  "summaryPath": "./.opencode/sessions/",
  "autoRestart": true
}
EOF
fi

# 4. 의존성 설치
echo "📥 Installing dependencies..."
cd "$TARGET_DIR"
npm install --production

echo ""
echo "✅ Installation complete!"
echo "🚀 The plugin is installed in the standard OpenCode plugin directory."
echo "💡 No need to add it to opencode.json. It will be loaded automatically."
echo "⚙️  You can customize settings in $TARGET_DIR/ralph-loop.json"
echo "   or create a ralph-loop.json in your project root for project-specific settings."
