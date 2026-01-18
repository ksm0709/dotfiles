# Design: Todo List 기반 워크플로우 표준화

## 1. 도구 설정 (Tool Configuration)
모든 에이전트는 계획 수립과 추적을 위해 다음 도구에 대한 접근 권한이 필요합니다.
```yaml
tools:
  todowrite: true
  todoread: true
```

## 2. 지침 추가 (Instruction Updates)
각 에이전트의 역할 정의(Role) 또는 워크플로우(Workflow) 섹션 최상단에 다음 내용을 표준화하여 추가합니다.

### 표준 지침 문구
```markdown
## 작업 관리 원칙 (Task Management)

1. **Todo List 작성**: 작업을 시작하기 전, 반드시 `todowrite` 도구를 사용하여 수행할 작업의 목록을 작성하세요.
2. **상태 업데이트**: 작업이 진행됨에 따라 Todo List의 상태(`pending` -> `in_progress` -> `completed`)를 실시간으로 업데이트하세요.
3. **동적 조정**: 새로운 발견이나 요구사항 변경이 발생하면 Todo List를 수정하여 반영하세요.
```

## 3. 파일별 적용 전략

| 파일명 | 적용 위치 | 비고 |
| :--- | :--- | :--- |
| `pm.md` | `핵심 원칙` 섹션 수정 | 이미 도구 보유 중 |
| `agent-template.md` | `워크플로우` 섹션의 `1. 계획` 단계 강화 | 템플릿 업데이트 |
| `senior-sw-engineer.md` | `워크플로우` 도입부 추가 | 도구 추가 필요 가능성 |
| `research-analyst.md` | `워크플로우`의 `1. 주제 분석` 단계와 통합 | 도구 추가 필요 가능성 |
| 그 외 에이전트 | `Role` 설명 직후 또는 `Core Principles` 추가 | 도구 추가 및 지침 삽입 |

## ADDED Requirements

### 공통 요구사항
- 모든 에이전트는 `todowrite`와 `todoread` 도구를 사용할 수 있어야 한다.
- 에이전트 프롬프트에 "작업 시작 전 Todo List 작성"이 명시되어야 한다.
- 에이전트 프롬프트에 "진행 상황에 따른 Todo List 업데이트"가 명시되어야 한다.

#### Scenario: 복잡한 리팩토링 요청 시
1. 사용자가 `senior-sw-engineer`에게 "결제 모듈 리팩토링해줘" 요청
2. 에이전트가 `todowrite`로 [코드 분석, 테스트 작성, 리팩토링, 검증] 등의 Todo List 생성
3. 코드 분석 완료 후 첫 번째 항목을 `completed`로 마킹하고 다음 항목 진행
