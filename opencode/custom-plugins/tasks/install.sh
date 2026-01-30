#!/bin/bash

# Tasks Plugin Installer for OpenCode
# 
# OpenCode 공식 플러그인 구조에 맞게 설치합니다.
# 이 플러그인은 에이전트들이 작업을 체계적으로 관리할 수 있도록 하는 
# tasks_* 도구들을 제공합니다.
#
# 설치 구조:
#   ~/.config/opencode/
#   ├── plugins/tasks/          # 실행용 TypeScript 파일
#   │   ├── index.ts
#   │   ├── commands/
#   │   ├── lib/
#   │   └── types/
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

# 1. 기존 잘못 설치된 폴더 정리
OLD_DIR="$CONFIG_DIR/plugin/$PLUGIN_NAME"
if [ -d "$OLD_DIR" ]; then
    echo -e "${YELLOW}🧹 Cleaning up old installation at $OLD_DIR...${NC}"
    rm -rf "$OLD_DIR"
fi

# 2. 필요한 디렉토리 생성
echo -e "${BLUE}📁 Creating necessary directories...${NC}"
mkdir -p "$PLUGINS_DIR/$PLUGIN_NAME"
mkdir -p "$SHARED_DIR/$PLUGIN_NAME/docs"
mkdir -p "$SHARED_DIR/$PLUGIN_NAME/templates"

# 3. 플러그인 소스 파일 복사 (plugins/tasks/)
echo -e "${BLUE}📄 Copying plugin source files to $PLUGINS_DIR/$PLUGIN_NAME/...${NC}"

if [ -d "$SOURCE_DIR/src" ]; then
    # Copy all source files maintaining directory structure
    for dir in commands lib types; do
        if [ -d "$SOURCE_DIR/src/$dir" ]; then
            cp -r "$SOURCE_DIR/src/$dir" "$PLUGINS_DIR/$PLUGIN_NAME/"
            echo -e "${GREEN}  ✓ Copied $dir/${NC}"
        fi
    done
    
    # Copy index.ts as entry point
    cp "$SOURCE_DIR/src/index.ts" "$PLUGINS_DIR/$PLUGIN_NAME/"
    echo -e "${GREEN}  ✓ Copied index.ts${NC}"
    
    echo -e "${GREEN}✓ Plugin source copied to $PLUGINS_DIR/$PLUGIN_NAME/${NC}"
else
    echo -e "${RED}❌ Source directory not found: $SOURCE_DIR/src${NC}"
    exit 1
fi

# 4. 문서 파일 복사 (shared/tasks/)
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

echo -e "${GREEN}✓ Documentation copied to $SHARED_DIR/$PLUGIN_NAME/${NC}"

# 5. 의존성을 ~/.config/opencode/package.json에 추가 (기본 설치 시에만)
PACKAGE_JSON="$CONFIG_DIR/package.json"

if [ -z "$TARGET_DIR" ]; then
    # Default installation - update package.json
    echo -e "${BLUE}📥 Updating dependencies in $PACKAGE_JSON...${NC}"

    if [ ! -f "$PACKAGE_JSON" ]; then
        echo -e "${YELLOW}Creating $PACKAGE_JSON...${NC}"
        echo '{"dependencies": {}}' > "$PACKAGE_JSON"
    fi

    # jq가 있으면 사용, 없으면 경고
    if command -v jq &> /dev/null; then
        TEMP_FILE=$(mktemp)
        jq '.dependencies += {
          "uuid": "^11.1.0"
        }' "$PACKAGE_JSON" > "$TEMP_FILE" && mv "$TEMP_FILE" "$PACKAGE_JSON"
        echo -e "${GREEN}✓ Dependencies updated using jq${NC}"
    else
        echo -e "${YELLOW}⚠️  jq not found. Please manually add 'uuid': '^11.1.0' to $PACKAGE_JSON${NC}"
    fi
else
    # Isolated test mode - skip package.json update
    echo -e "${YELLOW}⚠️  Skipping package.json update (isolated test mode)${NC}"
fi

# 6. AGENTS.md 업데이트 (기본 설치 시에만)
AGENTS_MD="$CONFIG_DIR/AGENTS.md"
TASKS_GUIDE_TEMPLATE="$SHARED_DIR/$PLUGIN_NAME/templates/agents-md-tasks-guide.md"
UPDATE_SCRIPT="$SOURCE_DIR/scripts/update-agents-md.py"

if [ -z "$TARGET_DIR" ]; then
    # Default installation - update AGENTS.md
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
    echo -e "${YELLOW}⚠️  Skipping AGENTS.md update (isolated test mode)${NC}"
fi

echo ""
echo -e "${GREEN}✅ Installation complete!${NC}"
echo ""
echo -e "${BLUE}📁 Installation Structure:${NC}"
echo "  • Plugin source: $PLUGINS_DIR/$PLUGIN_NAME/"
echo "    - index.ts (entry point)"
echo "    - commands/ (command implementations)"
echo "    - lib/ (utility libraries)"
echo "    - types/ (TypeScript type definitions)"
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
    echo "  2. Add tasks tools to your agent's frontmatter:"
    echo "     tools:"
    echo "       tasks_*: true"
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
