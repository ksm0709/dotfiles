# Change: Allow Todo Tools for All Subagents

## Why
모든 에이전트가 작업 계획을 체계적으로 관리하고 사용자에게 진행 상황을 투명하게 공유할 수 있도록, `todowrite` 및 `todoread` 도구를 글로벌 표준 도구로 명시하고 모든 에이전트에게 허용합니다.

## What Changes
- `opencode/config/AGENTS.md`의 **Global Tools & Skills** 섹션에 `todowrite` 및 `todoread`를 추가합니다.
- `opencode/config/templates/user-agents-template.md`에도 동일하게 추가하여 향후 생성되는 설정 파일에 반영되도록 합니다.
- (이미 대부분의 에이전트 파일에 포함되어 있으나) 모든 에이전트 설정 파일의 `tools` 섹션에 해당 도구가 활성화되어 있는지 확인하고 보완합니다.

## Impact
- **Affected specs**: `agent-config`
- **Affected files**: 
    - `opencode/config/AGENTS.md`
    - `opencode/config/templates/user-agents-template.md`
- **Behavior**: 모든 에이전트가 `todowrite`, `todoread` 도구를 공식적으로 사용할 수 있게 됩니다.
