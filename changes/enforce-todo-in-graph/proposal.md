# OpenSpec Proposal: 워크플로우 그래프 내 Todo 작성 명시화

## 1. 배경 및 목적 (Background & Objectives)
에이전트들이 작업 시작 시 Todo List를 작성하도록 텍스트 지침을 추가했으나, 워크플로우의 시각적 표현(Mermaid 그래프)에는 이 단계가 명시되지 않아 간과될 가능성이 있습니다.
이 제안은 `pm.md`와 `agent-template.md`의 Mermaid 그래프 시작점에 Todo 작성 단계를 명시적으로 추가하여, 계획 수립의 중요성을 시각적으로 강조하고 실행력을 높이는 것을 목표로 합니다.

## 2. 변경 범위 (Scope of Changes)
- **대상 파일**:
    - `opencode/config/agent/pm.md`
    - `opencode/config/agent/agent-template.md`
- **변경 내용**:
    - Mermaid 그래프의 `Start` 노드 직후에 `Todo` 작성 단계를 추가하거나, 첫 번째 단계의 라벨을 수정하여 Todo 작성을 포함시킴.

## 3. 기대 효과 (Expected Benefits)
- 에이전트가 워크플로우 그래프만 보고도 Todo 작성의 필요성을 인지.
- 작업 계획 수립 단계의 가시성 확보.
- 에이전트 템플릿을 사용하는 신규 에이전트들의 표준 준수율 향상.

## 4. 리스크 (Risks)
- 그래프가 복잡해질 수 있으나, 명확성을 위해 감수할 만한 수준임.
