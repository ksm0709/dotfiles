# Changes Agent Configuration (AGENTS.md)

이 파일은 **Changes** 변경 제안 관리 시스템의 AI 에이전트 설정을 정의합니다. 프로젝트 변경 제안, 설계, 구현 관리에 대한 문맥, 구조, 규칙을 명시합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: 프로젝트 변경 제안 및 설계 문서 관리 시스템
- **Goals**: 체계적인 변경 관리, 기술적 의사결정, 구현 계획 수립
- **Architecture**: 변경 항목별 디렉토리 구조 (proposal/design/tasks)
- **Domain Knowledge**: 변경 관리, 기술 설계, 프로젝트 관리, 문서화

### Tech Stack (기술 스택)
- **Language**: Markdown
- **Framework**: OpenSpec 기반의 변경 제안 프로세스
- **Documentation**: Markdown 기반의 정형화된 문서
- **Infrastructure**: Git 기반의 버전 관리

---

## 2. Folder Structure (폴더 구조)

```
[changes]/
├── [change-name]/         # 개별 변경 항목 디렉토리
│   ├── proposal.md        # 변경 제안서
│   ├── design.md          # 상세 설계 문서
│   └── tasks.md           # 구현 작업 목록
├── create-agent-system/   # 에이전트 시스템 생성 변경
├── enforce-todo-in-graph/ # Todo 그래프 강제화 변경
├── enforce-todo-workflow/ # Todo 워크플로우 강제화 변경
├── refactor-agent-template/ # 에이전트 템플릿 리팩토링 변경
├── refactor-all-agents-with-guidelines/ # 전체 에이전트 가이드라인 리팩토링 변경
├── standardize-agents/   # 에이전트 표준화 변경
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Markdown 가이드라인, OpenSpec 표준
- **Naming Convention**: 
  - 변경 디렉토리: kebab-case (예: create-agent-system)
  - 파일명: 소문자 및 하이픈 (예: proposal.md)
- **Formatting**: 정형화된 문서 구조 따르기
- **Linting**: MarkdownLint 사용 권장

### Security (보안)
- **Authentication**: N/A (문서 관리 시스템)
- **Input Validation**: 모든 변경 제안은 기술적 타당성 검증 필요
- **Secrets**: 민감한 정보는 문서에 직접 포함하지 않음
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.

---

## 4. Guardrails (가드레일)

- **Input Safety**: 변경 제안서 주입 공격 방지를 위해 사용자 입력을 검증합니다.
- **Output Safety**: 프로젝트의 민감한 정보나 내부 데이터를 무분별하게 출력하지 않습니다.
- **Hallucination**: 기술적 설계에 대한 사실이 아닌 정보를 생성하지 않도록 주의합니다.
- **Destructive Actions**: 프로젝트 전체에 영향을 주는 변경은 신중하게 처리하고 검토를 거칩니다.

---

## 5. Workflows (워크플로우)

### Change Proposal Workflow
1. **Proposal**: 변경 제안서 작성 (proposal.md)
2. **Review**: 기술적 타당성 및 영향 분석
3. **Design**: 상세 설계 문서 작성 (design.md)
4. **Tasks**: 구현 작업 목록 작성 (tasks.md)
5. **Implementation**: 작업 수행 및 검증
6. **Final Review**: 결과 검토 및 반영

### Documentation Standards
- **Proposal.md**: 변경 배경, 목표, 영향 범위, 예상 결과
- **Design.md**: 기술적 설계, 아키텍처, 구현 방법
- **Tasks.md**: 구체적인 작업 목록, 우선순위, 담당자

---

## 6. Environment Variables (환경 변수)

- `CHANGES_DIR`: Changes 디렉토리 경로
- `OPENSPEC_ENABLED`: OpenSpec 사용 여부
- `REVIEW_REQUIRED`: 변경 검토 필수 여부

---

## 7. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.