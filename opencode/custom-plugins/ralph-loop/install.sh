#!/bin/bash

# Ralph Loop Plugin Installer for OpenCode
# 
# OpenCode 공식 플러그인 구조에 맞게 설치합니다.
# 참고: https://opencode.ai/docs/plugins
#
# 핵심 변경사항:
# - 플러그인 디렉토리: ~/.config/opencode/plugins/ (복수형)
# - 플러그인 파일: 단일 .ts 파일로 직접 배치
# - 의존성: ~/.config/opencode/package.json에 추가

set -e

PLUGIN_NAME="ralph-loop"
CONFIG_DIR="$HOME/.config/opencode"
PLUGINS_DIR="$CONFIG_DIR/plugins"  # 복수형 (OpenCode 공식 문서 기준)
SOURCE_DIR=$(dirname "$(readlink -f "$0")")

echo "📦 Installing $PLUGIN_NAME plugin..."
echo ""

# 1. 기존 잘못 설치된 폴더 정리 (단수형 plugin 폴더 내 ralph-loop)
OLD_DIR="$CONFIG_DIR/plugin/$PLUGIN_NAME"
if [ -d "$OLD_DIR" ]; then
    echo "🧹 Cleaning up old installation at $OLD_DIR..."
    rm -rf "$OLD_DIR"
fi

# 2. 올바른 plugins 디렉토리 생성 (복수형)
mkdir -p "$PLUGINS_DIR"

# 3. 플러그인 파일 복사 (단일 .ts 파일로 직접 배치)
echo "📄 Copying plugin file to $PLUGINS_DIR/$PLUGIN_NAME.ts..."
cp "$SOURCE_DIR/src/index.ts" "$PLUGINS_DIR/$PLUGIN_NAME.ts"

# 4. 의존성을 ~/.config/opencode/package.json에 추가
PACKAGE_JSON="$CONFIG_DIR/package.json"
echo "📥 Updating dependencies in $PACKAGE_JSON..."

if [ ! -f "$PACKAGE_JSON" ]; then
    echo "Creating $PACKAGE_JSON..."
    echo '{"dependencies": {}}' > "$PACKAGE_JSON"
fi

# jq가 있으면 사용, 없으면 수동으로 처리
if command -v jq &> /dev/null; then
    # zod 의존성 추가 (이미 @opencode-ai/plugin은 있을 것으로 예상)
    TEMP_FILE=$(mktemp)
    jq '.dependencies += {"zod": "^4.3.6", "@opencode-ai/sdk": "^1.1.35"}' "$PACKAGE_JSON" > "$TEMP_FILE" && mv "$TEMP_FILE" "$PACKAGE_JSON"
else
    # jq가 없으면 node로 처리
    if command -v node &> /dev/null; then
        node -e "
const fs = require('fs');
const pkg = JSON.parse(fs.readFileSync('$PACKAGE_JSON', 'utf8'));
pkg.dependencies = pkg.dependencies || {};
pkg.dependencies['zod'] = '^4.3.6';
pkg.dependencies['@opencode-ai/sdk'] = '^1.1.35';
fs.writeFileSync('$PACKAGE_JSON', JSON.stringify(pkg, null, 2));
"
    else
        echo "⚠️  Neither jq nor node found. Please manually add 'zod' and '@opencode-ai/sdk' to $PACKAGE_JSON"
    fi
fi

# 5. 설정 파일 생성 (없을 경우에만)
GLOBAL_CONFIG="$CONFIG_DIR/$PLUGIN_NAME.json"
if [ ! -f "$GLOBAL_CONFIG" ]; then
    echo "📝 Creating default config at $GLOBAL_CONFIG..."
    cat <<EOF > "$GLOBAL_CONFIG"
{
  "promiseWord": "DONE",
  "maxRetries": 5,
  "summaryPath": "./.opencode/sessions/",
  "autoRestart": true
}
EOF
fi

echo ""
echo "✅ Installation complete!"
echo ""
echo "📁 Plugin installed to: $PLUGINS_DIR/$PLUGIN_NAME.ts"
echo "📦 Dependencies added to: $PACKAGE_JSON"
echo "⚙️  Config file: $GLOBAL_CONFIG"
echo ""
echo "💡 OpenCode will automatically load the plugin on next startup."
echo "   Dependencies will be installed via 'bun install' by OpenCode."
echo ""
echo "🔧 You can customize settings in:"
echo "   1. {project-dir}/.opencode/ralph-loop.json (Project-specific)"
echo "   2. ~/.config/opencode/ralph-loop.json (User global)"
