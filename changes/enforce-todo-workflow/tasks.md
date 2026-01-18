# Tasks: Todo List 워크플로우 적용

## 1. 도구 권한 추가
- [ ] `opencode/config/agent/*.md` 파일을 순회하며 `tools` 섹션 확인
- [ ] `todowrite: true`, `todoread: true`가 없는 파일에 추가

## 2. 프롬프트 지침 업데이트
- [ ] `pm.md`: 핵심 원칙에 Todo List 관련 내용 보강
- [ ] `agent-template.md`: 템플릿의 계획 단계에 Todo List 작성 명시
- [ ] `senior-sw-engineer.md`: 작업 관리 원칙 섹션 추가
- [ ] `py-code-reviewer.md`: 리뷰 절차에 체크리스트 작성 단계 추가
- [ ] `research-analyst.md`: 리서치 계획 단계에서 Todo List 활용 명시
- [ ] `agent-creator.md`: 에이전트 생성 절차에 Todo List 활용 명시
- [ ] `general.md`, `build.md`, `plan.md`, `cpp-review.md`: 작업 관리 원칙 섹션 추가

## 3. 검증
- [ ] 각 에이전트 파일의 문법(YAML Frontmatter 등) 유효성 검사
- [ ] 변경된 에이전트가 Todo List를 생성하는지 샘플 테스트 (시뮬레이션)
