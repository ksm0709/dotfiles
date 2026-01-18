# OpenSpec Proposal: 에이전트 설정 파일 구조 표준화

## 1. 배경 및 목적 (Background & Objectives)
현재 `opencode/config/agent/` 내의 에이전트 파일들은 서로 다른 구조와 형식을 가지고 있어 일관성이 부족합니다. 최근 도입된 `agent-template.md`를 기준으로 모든 에이전트 파일의 구조를 통일하여 유지보수성을 높이고, 에이전트 간의 동작 방식(특히 워크플로우 및 Todo 관리)을 표준화하고자 합니다.

## 2. 변경 범위 (Scope of Changes)
- **대상 파일**: `opencode/config/agent/*.md` (template 및 creator 제외)
- **주요 변경 사항**:
    - 모든 에이전트 파일에 `Role`, `Core Principles`, `Workflow` (Mermaid 포함), `Reference` 섹션 구조 적용.
    - 기존의 `Goals`, `Scope`, `Guidelines` 등의 내용을 표준 섹션으로 재배치.
    - `Task Management` (Todo List) 지침을 `Workflow`의 `Plan` 단계로 통합.

## 3. 기대 효과 (Expected Benefits)
- 에이전트 설정의 가독성 및 일관성 향상.
- 새로운 기능(예: 워크플로우 강제) 적용 시 일괄 적용 용이.
- 에이전트가 자신의 역할과 절차를 더 명확히 인지하여 성능 향상 기대.

## 4. 리스크 (Risks)
- 기존의 잘 작동하던 프롬프트(지침)가 재배치 과정에서 누락되거나 강조점이 희석될 수 있음. (기존 내용을 최대한 보존하며 구조만 변경하는 전략으로 완화)
