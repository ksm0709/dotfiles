# OpenSpec Proposal: 에이전트 템플릿 개선 및 위치 변경

## 1. 배경 및 목적 (Background & Objectives)
현재 `agent-template.md` 파일이 에이전트 설정 디렉토리(`opencode/config/agent/`)에 위치하여 실제 에이전트로 오인되어 설치되는 문제가 있습니다. 또한, 템플릿 내의 대괄호(`[ ]`)가 YAML 파싱 오류를 유발하고 있으며, 가이드라인 섹션이 부족하여 표준화된 에이전트 정의에 한계가 있습니다.
이 제안은 템플릿의 위치를 이동하고 내용을 보강하여 이러한 문제를 해결하는 것을 목표로 합니다.

## 2. 변경 범위 (Scope of Changes)
- **파일 이동**: `opencode/config/agent/agent-template.md` -> `opencode/config/agent-template.md`
- **내용 수정**:
    - `agent-template.md`: 제목의 대괄호 제거, `Guidelines` 섹션(Boundary, Command, Conventions) 추가.
    - `opencode/config/agent/agent-creator.md`: 템플릿 참조 경로 수정.
- **환경 정리**:
    - 설치 스크립트 실행 후 잘못 설치된 `~/.config/opencode/agent/agent-template.md` 제거.

## 3. 기대 효과 (Expected Benefits)
- 템플릿이 실제 에이전트로 잘못 인식되는 문제 해결.
- YAML 파싱 오류 방지.
- 에이전트 생성 시 더 명확하고 상세한 가이드라인 제공 가능.

## 4. 리스크 (Risks)
- `agent-creator`가 이전 경로를 참조하고 있을 경우 에이전트 생성 실패 가능성 (경로 수정으로 해결).
