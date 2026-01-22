# Change: Update Testing Policy

## Why
테스트 실행 시 프로젝트 파일 시스템을 오염시키거나 의도치 않은 부작용을 방지하기 위해, 모든 테스트는 격리된 환경에서 수행되어야 합니다. 이는 테스트의 신뢰성을 높이고 안전한 개발 환경을 보장합니다.

## What Changes
- `opencode/config/AGENTS.md`의 **Global Principles** 섹션에 "테스트 격리 원칙"을 추가합니다.
- 모든 테스트 코드는 시스템 임시 디렉토리를 생성하여 해당 공간 내에서 파일 조작 등을 수행해야 함을 명시합니다.

## Impact
- **Affected specs**: `agent-config` (New)
- **Affected files**: `opencode/config/AGENTS.md`
- **Behavior**: 향후 모든 에이전트는 테스트 코드 작성 시 `tempfile` 모듈 등을 활용하여 격리된 디렉토리를 사용해야 합니다.
