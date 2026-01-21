# Global Agent Configuration (AGENTS.md)

이 파일은 **사용자(User)** 레벨의 글로벌 AI 에이전트 설정을 정의합니다. 모든 프로젝트에 공통적으로 적용되는 원칙, 사용자 선호도, 글로벌 도구 및 에이전트를 관리합니다.

---

## 1. Global Principles (핵심 원칙)

모든 작업에서 기본적으로 준수해야 할 원칙입니다.

1.  **MUST** **한국어 소통**: 모든 대화, 주석, 문서는 **한국어**를 기본으로 합니다. (기술 용어는 영어 사용 가능)
2.  **MUST** **안전 우선**: 파일 삭제, 강제 푸시(`git push -f`) 등 파괴적인 작업은 반드시 사용자의 명시적 승인을 받습니다.
3.  **MUST** **검증 필수**: 코드를 작성한 후에는 반드시 실행하거나 테스트하여 동작을 검증해야 합니다.
4.  **MUST** **컨텍스트 유지**: 작업의 전후 맥락을 파악하고, 불필요한 중복 질문을 하지 않습니다.
5.  **MUST** **TDD 준수**: 기능 구현 시 테스트 코드를 먼저 작성하거나(TDD), 구현과 동시에 테스트를 작성하여 신뢰성을 보장합니다.

### Security (보안 원칙)
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.
- **Parameterized queries only**: SQL 인젝션 방지를 위해 반드시 파라미터화된 쿼리를 사용합니다.

---

## 2. User Preferences (사용자 선호)

사용자의 작업 스타일과 선호도 설정입니다.

- **Preferred Language**: [선호하는 프로그래밍 언어 (예: Python, TypeScript)]
- **Editor**: [사용하는 에디터 (예: VS Code, Vim)]
- **Communication Style**: [선호하는 소통 방식 (예: 간결하게, 상세하게, 예시 위주)]
- **Code Style**: [선호하는 코드 스타일 (예: 함수형, 객체지향)]

---

## 3. Global Tools & Skills (글로벌 도구 및 스킬)

어떤 프로젝트에서든 사용할 수 있는 전역 도구와 스킬입니다.

### 🛠️ Standard Tools
- `bash`: 터미널 명령어 실행
- `read`/`write`/`edit`: 파일 조작
- `glob`/`grep`: 파일 검색
- `webfetch`: 웹 콘텐츠 가져오기

### 🧠 Specialized Skills
- **deep-research**: 심층 웹 리서치 및 보고서 생성 (`/deep-research`)
- **memory**: 장기 기억 관리 및 검색
- **doc-coauthoring**: 문서 공동 작성 지원
- **agent-creator**: 새로운 에이전트 생성 및 설정 (`/create-agent`)

---

## 4. Global Agent Registry (글로벌 에이전트 명부)

전역적으로 호출 가능한 에이전트 목록입니다.

### 📋 Project Manager (PM)
- **File**: `.config/opencode/agent/pm.md`
- **Role**: 개발 PM
- **Trigger**: 복잡한 기획, 스펙 정의 필요 시

### 👨‍💻 Senior SW Engineer (Dev)
- **File**: `.config/opencode/agent/senior-sw-engineer.md`
- **Role**: 시니어 개발자
- **Trigger**: 코드 구현, 리팩토링, 테스트 작성 시

### 🐍 Py Code Reviewer
- **File**: `.config/opencode/agent/py-code-reviewer.md`
- **Role**: Python 코드 리뷰어
- **Trigger**: Python 코드 품질 검토 시

### 🛠️ Agent Creator
- **File**: `.config/opencode/agent/agent-creator.md`
- **Role**: 에이전트 크리에이터
- **Trigger**: 새로운 에이전트 생성 시 (`/create-agent`)

---

## 5. Integration (통합)

이 글로벌 설정은 프로젝트별 `AGENTS.md`와 함께 작동합니다.

- **우선순위**: 프로젝트별 `AGENTS.md`의 설정이 글로벌 설정보다 우선합니다.
- **상속**: 프로젝트 설정에 명시되지 않은 항목은 글로벌 설정을 따릅니다.
