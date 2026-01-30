# Global Agent Configuration (AGENTS.md)

이 파일은 **사용자(User)** 레벨의 글로벌 AI 에이전트 설정을 정의합니다. 모든 프로젝트에 공통적으로 적용되는 원칙, 사용자 선호도, 글로벌 도구 및 에이전트를 관리합니다.

---

## 1. Global Principles (핵심 원칙)

모든 작업에서 기본적으로 준수해야 할 원칙입니다.

1.  **MUST** **한국어 소통**: 모든 대화, 주석, 문서는 **한국어**를 기본으로 합니다. (기술 용어는 영어 사용 가능)
2.  **MUST** **안전 우선**: 파일 삭제, 강제 푸시(`git push -f`) 등 파괴적인 작업은 반드시 사용자의 명시적 승인을 받습니다.
3.  **MUST** **검증 필수**: 코드를 작성한 후에는 반드시 실행하거나 테스트하여 동작을 검증해야 합니다.
4.  **MUST** **컨텍스트 유지**: 작업의 전후 맥락을 파악하고, 불필요한 중복 질문을 하지 않습니다.
5.  **MUST** **TDD 준수**: 기능 구현 전 반드시 테스트 코드를 먼저 작성(TDD), 신뢰성을 보장합니다.
6.  **MUST** **테스트 격리**: 모든 테스트 코드는 프로젝트와 격리된 시스템 임시 디렉토리를 생성하여 진행해야 합니다.

### Security (보안 원칙)
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.
- **Parameterized queries only**: SQL 인젝션 방지를 위해 반드시 파라미터화된 쿼리를 사용합니다.

---

## 2. User Preferences (사용자 선호)

사용자의 작업 스타일과 선호도 설정입니다.

- **Preferred Language**: Python, TypeScript
- **Editor**: VS Code
- **Communication Style**: 명확하고 간결하게, 필요한 경우 예시 제공
- **Code Style**: PEP8 (Python), ESLint/Prettier (TypeScript)

---

## 3. Global Tools & Skills (글로벌 도구 및 스킬)

어떤 프로젝트에서든 사용할 수 있는 전역 도구와 스킬입니다.

### 🛠️ Standard Tools
- `bash`: 터미널 명령어 실행
- `read`/`write`/`edit`: 파일 조작
- `glob`/`grep`: 파일 검색
- `webfetch`: 웹 콘텐츠 가져오기
- `todowrite`/`todoread`: 작업 목록 관리 및 조회 (Todo List)

### 🧠 Specialized Skills
- **deep-research**: 심층 웹 리서치 및 보고서 생성 (`/deep-research`)
- **memory**: 장기 기억 관리 및 검색
- **doc-coauthoring**: 문서 공동 작성 지원
- **agent-creator**: 새로운 에이전트 생성 및 설정 (`/create-agent`)

---


<!-- TASKS_TOOLS_GUIDE_START -->
<!-- 이 섹션은 Tasks Plugin에 의해 자동으로 관리됩니다. 수동으로 수정하지 마세요. -->

### 📋 Task Management Tools

Tasks Plugin은 에이전트가 작업을 체계적으로 관리할 수 있도록 다음 도구들을 제공합니다:

#### 사용 가능한 도구 (tasks_*)

Tasks Plugin은 `tasks_*` 와일드카드로 모든 작업 관리 도구를 제공합니다.

**주요 도구:**
- **`tasks_init(agent, title)`**: 작업 목록 초기화
  - 반환값: `{ title, agent, fileName, taskIds, totalTasks }`
- **`tasks_list(format)`**: 작업 목록 조회 (format: markdown/json/table)
  - 반환값: `{ success, taskLists, formattedOutput, message }`
- **`tasks_update(id, status)`**: 작업 상태 업데이트 (status: pending/in_progress/completed)
  - 반환값: `{ success, taskId, status, message }`
- **`tasks_complete(id)`**: 작업 완료 처리
  - 반환값: `{ success, taskId, message }`
- **`tasks_add(title, parent)`**: 새 작업 추가 (parent는 선택적)
  - 반환값: `{ success, title, parent, details, message }`
- **`tasks_remove(id)`**: 작업 제거
  - 반환값: `{ success, taskId, taskTitle, message }`
- **`tasks_status()`**: 전체 진행 상황 확인
  - 반환값: `{ success, summaries, formattedOutput, message }`

**사용 예시:**
```typescript
// 작업 목록 초기화
const initResult = tasks_init(agent="senior-sw-engineer", title="API-구현")
// 응답 예시:
// ✅ Task list "API-구현" initialized successfully for agent "senior-sw-engineer"
// 📁 File: senior-sw-engineer-api-구현.md
// 📊 Total tasks: 0

// 작업 추가
tasks_add(title="요구사항 분석")
tasks_add(title="설계")
tasks_add(title="구현")

// 작업 상태 업데이트
tasks_update(id="1", status="in_progress")

// 작업 완료
tasks_complete(id="1")

// 진행 상황 확인
const statusResult = tasks_status()

// 작업 목록 조회
const listResult = tasks_list(format="markdown")
```

**주요 특짱:**
- 세션 ID는 OpenCode 컨텍스트에서 자동으로 추출됩니다 (에이전트가 입력할 필요 없음)
- 모든 도구는 반환값을 통해 결과를 전달합니다 (TUI 깨짐 없음)
- 작업 파일은 `~/.local/share/opencode/tasks/{session-id}/`에 저장됩니다

**설치 구조:**
- 플러그인 코드: `~/.config/opencode/plugins/tasks/`
- 문서 및 가이드: `~/.config/opencode/shared/tasks/`

#### 에이전트 설정

에이전트가 Tasks 도구를 사용하려면 frontmatter에 다음을 추가하세요:

**방법 1: `tasks_*` 와일드카드 사용 (권장)**
모든 Tasks 도구를 한 번에 활성화하려면 `tasks_*` 와일드카드를 사용하세요:

```yaml
---
tools:
  tasks_*: true
---
```

**방법 2: 개별 도구 활성화**
특정 도구만 필요한 경우 개별적으로 지정할 수 있습니다:

```yaml
---
tools:
  tasks_init: true
  tasks_list: true
  tasks_update: true
  tasks_complete: true
  tasks_add: true
  tasks_remove: true
  tasks_status: true
---
```

#### 자세한 사용법

자세한 사용법은 다음 문서를 참조하세요:
`~/.config/opencode/shared/tasks/docs/tasks-tools-guide.md`

<!-- TASKS_TOOLS_GUIDE_END -->










## 4. Global Agent Registry (글로벌 에이전트 명부)

전역적으로 호출 가능한 에이전트 목록입니다.

### 📋 Project Manager (PM)
- **File**: `.config/opencode/agent/pm.md`
- **Role**: 개발 PM (Project Manager)
- **Description**: 프로젝트 총괄, 기술적 의사결정, 스펙 정의(OpenSpec), 일정 관리.
- **Trigger**: 새로운 기능 개발, 복잡한 요구사항 분석, 프로젝트 구조 변경 시.

### 👨‍💻 Senior SW Engineer (Dev)
- **File**: `.config/opencode/agent/senior-sw-engineer.md`
- **Role**: Senior Software Engineer
- **Description**: 실제 코드 구현, 리팩토링, 테스트 작성 (다국어 지원).
- **Trigger**: 구체적인 기능 구현, 버그 수정, 단위 테스트 작성 시.

### 🐍 Py Code Reviewer
- **File**: `.config/opencode/agent/py-code-reviewer.md`
- **Role**: Py Code Reviewer
- **Description**: Python 코드 품질, PEP 표준, 보안 취약점 검토.
- **Trigger**: Python 코드 구현 완료 후 PR 생성 전.

### ➕ C++ Code Reviewer
- **File**: `.config/opencode/agent/cpp-review.md`
- **Role**: C++ Code Reviewer
- **Description**: C++ 코드 품질, 메모리 안전성, 성능 검토.
- **Trigger**: C++ 코드 리뷰 요청 시.

### 🏗️ Build Agent
- **File**: `.config/opencode/agent/build.md`
- **Role**: Build Agent
- **Description**: 프로젝트 빌드, 의존성 관리, 환경 설정.
- **Trigger**: 빌드 오류 해결, 패키지 설치, 환경 구성 시.

### 📅 Planner
- **File**: `.config/opencode/agent/plan.md`
- **Role**: Planner
- **Description**: 고수준 계획, 전략 수립, 로드맵 작성 (코드 수정 불가).
- **Trigger**: 프로젝트 초기 기획, 장기 로드맵 수립 시.

### 🔎 Research Analyst
- **File**: `.config/opencode/agent/research-analyst.md`
- **Role**: 리서치 분석가 (Research Analyst)
- **Description**: 심층 웹 검색 및 정보 분석, 리포트 작성.
- **Trigger**: 복잡한 주제 연구, 최신 기술 동향 파악 시.

### 🛠️ Agent Creator
- **File**: `.config/opencode/agent/agent-creator.md`
- **Role**: 에이전트 크리에이터 (Agent Creator)
- **Description**: 새로운 에이전트 설정 파일 생성.
- **Trigger**: 새로운 역할의 에이전트가 필요할 때.

### 🌐 General Agent
- **File**: `.config/opencode/agent/general.md`
- **Role**: General Agent
- **Description**: 범용 작업 처리 및 다단계 태스크 실행.
- **Trigger**: 특정 카테고리에 속하지 않는 복잡한 작업 시.

---

## 5. Integration (통합)

이 글로벌 설정은 프로젝트별 `AGENTS.md`와 함께 작동합니다.

- **우선순위**: 프로젝트별 `AGENTS.md`의 설정이 글로벌 설정보다 우선합니다.
- **상속**: 프로젝트 설정에 명시되지 않은 항목은 글로벌 설정을 따릅니다.












<!-- OPENCODE_MEMORY_INSTRUCTIONS_START -->
### Context & Memory Management (컨텍스트 및 기억 관리)
OpenCode의 장기 기억(Long-term Memory)과 작업 기억(Working Memory)을 적극 활용하여 일관성을 유지합니다.

**⚠️ 필수 워크플로우 (Mandatory Workflow)**
모든 작업은 반드시 다음 순서로 진행해야 합니다:
1. **Start**: `context_start`로 작업 시작 알림 (관련 기억 검색됨)
2. **Intent**: `context_intent`로 목표 설정 (사고 과정 기록)
3. **Work**: 작업 수행 (중간중간 `context_decision`, `context_learning` 기록)
4. **End**: `context_end`로 작업 종료 및 기억 저장

---

#### 🛠️ Memory Tools Guide

1. **Task Lifecycle (작업 수명주기)**
   - `context_start(task="...")`: 작업을 시작할 때 가장 먼저 호출합니다.
   - `context_end(result="...")`: 작업이 완료되면 반드시 호출하여 기억을 저장합니다.

2. **Thought Process (사고 과정)**
   - `context_intent`: 작업의 **목표(Goal)**와 **의도(Intent)**를 기록합니다. (작업 시작 직후 권장)
   - `context_decision`: 라이브러리 선택, 아키텍처 설계 등 **중요한 의사결정**을 기록합니다.
   - `context_learning`: 오류 해결, 새로운 패턴 발견 등 **학습한 내용**을 기록합니다.

3. **Maintenance (상태 관리)**
   - `context_checkpoint`: 작업이 길어지거나 단계가 바뀔 때 **중간 저장**을 수행합니다. (메모리 압축 효과)
   - `context_status`: 현재 작업의 진행 상태와 컨텍스트를 **점검**합니다.

이 도구들을 통해 에이전트는 과거의 경험을 기억하고 더 똑똑해집니다.
<!-- OPENCODE_MEMORY_INSTRUCTIONS_END -->
