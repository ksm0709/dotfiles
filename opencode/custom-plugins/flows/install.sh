#!/bin/bash

# Flows Plugin Installer for OpenCode
# 
# OpenCode 공식 플러그인 구조에 맞게 설치합니다.
# 참고: ralph-loop 플러그인의 설치 방식을 따릅니다.

set -e

PLUGIN_NAME="flows"
CONFIG_DIR="$HOME/.config/opencode"
PLUGINS_DIR="$CONFIG_DIR/plugins"
SOURCE_DIR=$(dirname "$(readlink -f "$0")")

echo "📦 Installing $PLUGIN_NAME plugin..."
echo ""

# 0. 기존 잘못 설치된 폴더 정리 (단수형 plugin 폴더 내 flows)
OLD_DIR="$CONFIG_DIR/plugin/$PLUGIN_NAME"
if [ -d "$OLD_DIR" ]; then
    echo "🧹 Cleaning up old installation at $OLD_DIR..."
    rm -rf "$OLD_DIR"
fi

# 1. 빌드 및 번들링
# flows 플러그인은 여러 파일로 구성되어 있으므로 단일 파일로 번들링이 필요합니다.
echo "🔨 Bundling plugin into a single file..."
mkdir -p "$SOURCE_DIR/dist"

# esbuild를 사용하여 번들링 (외부 의존성 제외)
"$SOURCE_DIR/node_modules/.bin/esbuild" "$SOURCE_DIR/src/index.ts" \
  --bundle \
  --platform=node \
  --format=esm \
  --outfile="$SOURCE_DIR/dist/$PLUGIN_NAME.js" \
  --external:@opencode-ai/plugin \
  --external:@opencode-ai/sdk \
  --external:zod \
  --external:uuid \
  --external:glob

# 2. 올바른 plugins 디렉토리 생성
mkdir -p "$PLUGINS_DIR"

# 3. 플러그인 파일 복사
# OpenCode는 plugins/ 폴더 내의 .ts 파일을 로드하므로 확장자를 .ts로 하여 복사합니다.
echo "📄 Copying bundled plugin to $PLUGINS_DIR/$PLUGIN_NAME.ts..."
cp "$SOURCE_DIR/dist/$PLUGIN_NAME.js" "$PLUGINS_DIR/$PLUGIN_NAME.ts"

# 4. 의존성을 ~/.config/opencode/package.json에 추가
PACKAGE_JSON="$CONFIG_DIR/package.json"
echo "📥 Updating dependencies in $PACKAGE_JSON..."

if [ ! -f "$PACKAGE_JSON" ]; then
    echo "Creating $PACKAGE_JSON..."
    echo '{"dependencies": {}}' > "$PACKAGE_JSON"
fi

# jq가 있으면 사용, 없으면 node로 처리
if command -v jq &> /dev/null; then
    TEMP_FILE=$(mktemp)
    jq '.dependencies += {
      "zod": "^3.25.67", 
      "@opencode-ai/sdk": "^1.1.35",
      "uuid": "^11.1.0",
      "glob": "^11.0.0"
    }' "$PACKAGE_JSON" > "$TEMP_FILE" && mv "$TEMP_FILE" "$PACKAGE_JSON"
else
    if command -v node &> /dev/null; then
        node -e "
const fs = require('fs');
const pkg = JSON.parse(fs.readFileSync('$PACKAGE_JSON', 'utf8'));
pkg.dependencies = pkg.dependencies || {};
pkg.dependencies['zod'] = '^3.25.67';
pkg.dependencies['@opencode-ai/sdk'] = '^1.1.35';
pkg.dependencies['uuid'] = '^11.1.0';
pkg.dependencies['glob'] = '^11.0.0';
fs.writeFileSync('$PACKAGE_JSON', JSON.stringify(pkg, null, 2));
"
    else
        echo "⚠️  Neither jq nor node found. Please manually add dependencies (zod, @opencode-ai/sdk, uuid, glob) to $PACKAGE_JSON"
    fi
fi

# 5. 설정 파일 생성 (없을 경우에만)
GLOBAL_CONFIG="$CONFIG_DIR/$PLUGIN_NAME.json"
if [ ! -f "$GLOBAL_CONFIG" ]; then
    echo "📝 Creating default config at $GLOBAL_CONFIG..."
    cat <<EOF > "$GLOBAL_CONFIG"
{
  "tickInterval": 500,
  "enableToasts": true,
  "keepCompletedInstances": false
}
EOF
fi

# 6. 플로우 정의 디렉토리 생성 및 예제 복사
FLOWS_DIR="$CONFIG_DIR/flows"
mkdir -p "$FLOWS_DIR"

EXAMPLES_DIR="$SOURCE_DIR/docs/examples"
if [ -d "$EXAMPLES_DIR" ]; then
    echo "📄 Copying example flows to $FLOWS_DIR..."
    cp "$EXAMPLES_DIR"/*.json "$FLOWS_DIR/"
fi

echo ""
echo "✅ Installation complete!"
echo ""
echo "📁 Plugin installed to: $PLUGINS_DIR/$PLUGIN_NAME.ts"
echo "📦 Dependencies added to: $PACKAGE_JSON"
echo "⚙️  Config file: $GLOBAL_CONFIG"
echo "📂 Flows directory: $CONFIG_DIR/flows/"
echo ""
echo "💡 OpenCode will automatically load the plugin on next startup."
echo "   Dependencies will be installed via 'bun install' by OpenCode."
echo ""
