# Design: 에이전트 구조 표준화 전략

## 1. 표준 구조 정의 (`agent-template.md` 기반)

### 섹션 순서
1. **Frontmatter**: 메타데이터 (변경 없음)
2. **Role**: 역할 정의 및 범위
3. **Core Principles**: 핵심 원칙 (한국어 소통, 품질, OpenSpec 등)
4. **Workflow**: Mermaid 차트 및 단계별 상세 Todo (Plan-Execute-Verify)
5. **Delegation**: (Optional) 서브 에이전트 위임 정보
6. **Reference/Guidelines**: (Optional) 상세 가이드라인, 안티패턴 등

## 2. 매핑 전략 (Mapping Strategy)

### `senior-sw-engineer.md`
- `Goals`, `Scope` -> `Role` 및 `Core Principles`
- `Task Management` -> `Workflow` > `Plan`
- `Development Workflow` -> `Workflow` (Mermaid 차트로 변환)
- `Guidelines`, `Testing Strategy`, `Anti-Patterns` -> `Reference` 하위 섹션으로 이동

### `py-code-reviewer.md` / `cpp-review.md`
- `Goals`, `Scope` -> `Role` 및 `Core Principles`
- `Review Process` -> `Workflow` (Mermaid 차트로 변환)
- `Review Focus Areas`, `Feedback Style` -> `Reference`

### `research-analyst.md`
- 이미 템플릿과 유사하므로 `Workflow` 섹션의 `Todo` 항목을 템플릿 형식(`- [ ]`)으로 구체화.

### `general.md`, `build.md`, `plan.md`
- 내용이 빈약하므로 템플릿의 기본 구조를 채워넣고, 각자의 역할에 맞는 간단한 워크플로우 정의.

## 3. 워크플로우 표준화
모든 에이전트는 최소한 다음 3단계를 포함해야 함:
1. **Plan**: `todowrite`를 사용한 계획 수립.
2. **Execute**: 본연의 작업 수행.
3. **Verify**: 결과 확인 및 완료 처리.

## ADDED Requirements
- 모든 에이전트는 Mermaid 문법으로 된 워크플로우 다이어그램을 포함해야 한다.
- `Core Principles`에 "한국어 소통"이 명시되어야 한다.
- `Workflow`의 첫 단계는 반드시 `todowrite` 사용을 포함해야 한다.

#### Scenario: 구조 변경 후 동작 확인
1. 사용자가 `senior-sw-engineer`에게 작업 요청.
2. 에이전트가 `Workflow` 섹션을 읽고 Mermaid 차트의 흐름을 이해.
3. `Plan` 단계의 지침에 따라 `todowrite`로 계획 작성.
4. `Reference` 섹션의 가이드라인을 준수하며 코드 작성.
