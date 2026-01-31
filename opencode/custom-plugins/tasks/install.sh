#!/bin/bash

# Tasks Plugin Installer for OpenCode
# 
# OpenCode 공식 플러그인 구조에 맞게 설치합니다.
# 참고: flows 플러그인의 번들링 방식을 따릅니다.
#
# 설치 구조:
#   ~/.config/opencode/
#   ├── plugins/tasks.ts        # 번들링된 단일 플러그인 파일
#   └── shared/tasks/           # 문서 및 가이드
#       ├── README.md
#       ├── docs/
#       └── templates/
#
# Usage:
#   ./install.sh                    # Install to default location (~/.config/opencode)
#   ./install.sh --target <path>    # Install to custom location (for isolated testing)
#   ./install.sh -t <path>          # Short form

set -e

PLUGIN_NAME="tasks"
SOURCE_DIR=$(dirname "$(readlink -f "$0")")

# Parse command line arguments
TARGET_DIR=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --target|-t)
            TARGET_DIR="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --target, -t <path>    Install to custom directory (for isolated testing)"
            echo "  --help, -h             Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                     # Install to ~/.config/opencode"
            echo "  $0 --target ./test-env # Install to ./test-env/.opencode for testing"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Set target directory
if [ -n "$TARGET_DIR" ]; then
    # Custom target directory (for isolated testing)
    mkdir -p "$TARGET_DIR/.opencode"
    CONFIG_DIR="$(cd "$TARGET_DIR/.opencode" && pwd)"
    echo -e "${BLUE}📦 Installing $PLUGIN_NAME plugin to custom location: $CONFIG_DIR${NC}"
    echo -e "${YELLOW}⚠️  Isolated test mode - will not update ~/.config/opencode/AGENTS.md${NC}"
else
    # Default location
    CONFIG_DIR="$HOME/.config/opencode"
    echo -e "${BLUE}📦 Installing $PLUGIN_NAME plugin...${NC}"
fi

PLUGINS_DIR="$CONFIG_DIR/plugins"
SHARED_DIR="$CONFIG_DIR/shared"

echo ""

# 1. 기존 잘못 설치된 폴더/파일 정리
echo -e "${BLUE}🧹 Cleaning up old installations...${NC}"

# 기존 단수형 plugin 폴더 정리
OLD_DIR="$CONFIG_DIR/plugin/$PLUGIN_NAME"
if [ -d "$OLD_DIR" ]; then
    echo -e "${YELLOW}  Removing old directory: $OLD_DIR${NC}"
    rm -rf "$OLD_DIR"
fi

# 기존 디렉토리 구조 정리 (복수형 plugins/tasks/)
OLD_PLUGIN_DIR="$PLUGINS_DIR/$PLUGIN_NAME"
if [ -d "$OLD_PLUGIN_DIR" ]; then
    echo -e "${YELLOW}  Removing old plugin directory: $OLD_PLUGIN_DIR${NC}"
    rm -rf "$OLD_PLUGIN_DIR"
fi

# 기존 tasks.ts 파일 정리 (v2 레거시)
OLD_PLUGIN_FILE="$PLUGINS_DIR/$PLUGIN_NAME.ts"
if [ -f "$OLD_PLUGIN_FILE" ]; then
    echo -e "${YELLOW}  Removing old plugin file: $OLD_PLUGIN_FILE${NC}"
    rm -f "$OLD_PLUGIN_FILE"
fi

echo -e "${GREEN}✓ Cleanup complete${NC}"
echo ""

# 2. 빌드 및 번들링
echo -e "${BLUE}🔨 Bundling plugin into a single file...${NC}"
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
  --external:uuid

echo -e "${GREEN}✓ Bundling complete${NC}"
echo ""

# 3. 올바른 plugins 디렉토리 생성
mkdir -p "$PLUGINS_DIR"

# 4. 플러그인 파일 복사
# OpenCode는 plugins/ 폴더 내의 .ts 파일을 로드하므로 확장자를 .ts로 하여 복사합니다.
echo -e "${BLUE}📄 Copying bundled plugin to $PLUGINS_DIR/$PLUGIN_NAME.ts...${NC}"
cp "$SOURCE_DIR/dist/$PLUGIN_NAME.js" "$PLUGINS_DIR/$PLUGIN_NAME.ts"
echo -e "${GREEN}✓ Plugin installed${NC}"
echo ""

# 5. 필요한 디렉토리 생성 (문서용)
echo -e "${BLUE}📁 Creating documentation directories...${NC}"
mkdir -p "$SHARED_DIR/$PLUGIN_NAME/docs"
mkdir -p "$SHARED_DIR/$PLUGIN_NAME/templates"

# 6. 문서 파일 복사 (shared/tasks/)
echo -e "${BLUE}📖 Copying documentation to $SHARED_DIR/$PLUGIN_NAME/...${NC}"

if [ -f "$SOURCE_DIR/README.md" ]; then
    cp "$SOURCE_DIR/README.md" "$SHARED_DIR/$PLUGIN_NAME/"
    echo -e "${GREEN}  ✓ Copied README.md${NC}"
fi

if [ -d "$SOURCE_DIR/docs" ]; then
    cp -r "$SOURCE_DIR/docs/"* "$SHARED_DIR/$PLUGIN_NAME/docs/"
    echo -e "${GREEN}  ✓ Copied docs/${NC}"
fi

if [ -d "$SOURCE_DIR/templates" ]; then
    cp -r "$SOURCE_DIR/templates/"* "$SHARED_DIR/$PLUGIN_NAME/templates/"
    echo -e "${GREEN}  ✓ Copied templates/${NC}"
fi

echo -e "${GREEN}✓ Documentation copied${NC}"
echo ""

# 7. 의존성을 ~/.config/opencode/package.json에 추가 (기본 설치 시에만)
PACKAGE_JSON="$CONFIG_DIR/package.json"

if [ -z "$TARGET_DIR" ]; then
    # Default installation - update package.json
    echo -e "${BLUE}📥 Updating dependencies in $PACKAGE_JSON...${NC}"

    if [ ! -f "$PACKAGE_JSON" ]; then
        echo -e "${YELLOW}Creating $PACKAGE_JSON...${NC}"
        echo '{"dependencies": {}}' > "$PACKAGE_JSON"
    fi

    # jq가 있으면 사용, 없으면 node로 처리
    if command -v jq &> /dev/null; then
        TEMP_FILE=$(mktemp)
        jq '.dependencies += {
          "uuid": "^11.1.0"
        }' "$PACKAGE_JSON" > "$TEMP_FILE" && mv "$TEMP_FILE" "$PACKAGE_JSON"
        echo -e "${GREEN}✓ Dependencies updated using jq${NC}"
    else
        if command -v node &> /dev/null; then
            node -e "
const fs = require('fs');
const pkg = JSON.parse(fs.readFileSync('$PACKAGE_JSON', 'utf8'));
pkg.dependencies = pkg.dependencies || {};
pkg.dependencies['uuid'] = '^11.1.0';
fs.writeFileSync('$PACKAGE_JSON', JSON.stringify(pkg, null, 2));
"
            echo -e "${GREEN}✓ Dependencies updated using node${NC}"
        else
            echo -e "${YELLOW}⚠️  Neither jq nor node found. Please manually add 'uuid': '^11.1.0' to $PACKAGE_JSON${NC}"
        fi
    fi
else
    # Isolated test mode - skip package.json update
    echo -e "${YELLOW}⚠️  Skipping package.json update (isolated test mode)${NC}"
fi

# 8. AGENTS.md 업데이트 (기본 설치 시에만)
AGENTS_MD="$CONFIG_DIR/AGENTS.md"
TASKS_GUIDE_TEMPLATE="$SHARED_DIR/$PLUGIN_NAME/templates/agents-md-tasks-guide.md"
UPDATE_SCRIPT="$SOURCE_DIR/scripts/update-agents-md.py"

if [ -z "$TARGET_DIR" ]; then
    # Default installation - update AGENTS.md
    echo ""
    echo -e "${BLUE}📝 Updating $AGENTS_MD with tasks tools guide...${NC}"

    if [ -f "$AGENTS_MD" ] && [ -f "$TASKS_GUIDE_TEMPLATE" ] && [ -f "$UPDATE_SCRIPT" ]; then
        python3 "$UPDATE_SCRIPT" "$AGENTS_MD" "$TASKS_GUIDE_TEMPLATE"
        echo -e "${GREEN}✓ AGENTS.md updated successfully${NC}"
    elif [ ! -f "$UPDATE_SCRIPT" ]; then
        echo -e "${YELLOW}⚠️  Update script not found at $UPDATE_SCRIPT. Skipping AGENTS.md update.${NC}"
    elif [ ! -f "$TASKS_GUIDE_TEMPLATE" ]; then
        echo -e "${YELLOW}⚠️  Template file not found at $TASKS_GUIDE_TEMPLATE. Skipping AGENTS.md update.${NC}"
    else
        echo -e "${YELLOW}⚠️  $AGENTS_MD not found. Skipping AGENTS.md update.${NC}"
    fi
else
    # Isolated test mode - skip AGENTS.md update
    echo ""
    echo -e "${YELLOW}⚠️  Skipping AGENTS.md update (isolated test mode)${NC}"
fi

echo ""
echo -e "${GREEN}✅ Installation complete!${NC}"
echo ""
echo -e "${BLUE}📁 Installation Structure:${NC}"
echo "  • Plugin: $PLUGINS_DIR/$PLUGIN_NAME.ts (bundled single file)"
echo "  • Documentation: $SHARED_DIR/$PLUGIN_NAME/"
echo "    - README.md"
echo "    - docs/tasks-tools-guide.md"
echo "    - templates/agents-md-tasks-guide.md"
if [ -z "$TARGET_DIR" ]; then
    echo "  • AGENTS.md updated with tasks tools guide"
fi
echo ""

if [ -z "$TARGET_DIR" ]; then
    echo -e "${BLUE}📦 Dependencies:${NC}"
    echo "  • Updated in: $PACKAGE_JSON"
    echo ""
    echo -e "${GREEN}💡 OpenCode will automatically load the plugin on next startup.${NC}"
    echo -e "${GREEN}   Dependencies will be installed via 'bun install' by OpenCode.${NC}"
    echo ""
    echo -e "${BLUE}📖 Next Steps:${NC}"
    echo "  1. Restart OpenCode to load the plugin"
    echo "  2. Add tasks tool to your agent's frontmatter:"
    echo "     tools:"
    echo "       tasks: true"
    echo "  3. See the guide: $SHARED_DIR/$PLUGIN_NAME/docs/tasks-tools-guide.md"
else
    echo -e "${YELLOW}⚠️  Isolated test mode - plugin installed to: $CONFIG_DIR${NC}"
    echo ""
    echo -e "${BLUE}📖 To test with OpenCode:${NC}"
    echo "  1. Set OPENCODE_CONFIG_DIR environment variable:"
    echo "     export OPENCODE_CONFIG_DIR=$CONFIG_DIR"
    echo "  2. Or run opencode with: --config-dir $CONFIG_DIR"
    echo "  3. Test the plugin in isolation without affecting your main setup"
fi

echo ""
