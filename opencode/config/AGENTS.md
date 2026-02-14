# Global Agent Configuration (AGENTS.md)

이 파일은 **사용자(User)** 레벨의 글로벌 AI 에이전트 설정을 정의합니다. 모든 프로젝트에 공통적으로 적용되는 원칙, 사용자 선호도, 글로벌 도구 및 에이전트를 관리합니다.

---

## 1. Global Principles (핵심 원칙)

모든 작업에서 기본적으로 준수해야 할 원칙입니다.

1.  **MUST** **한국어 소통, 영문 코딩**: 모든 대화, 문서는 **한국어**를 기본으로 합니다. (기술 용어는 영어 사용 가능) 모든 코드, 주석은 **영어** 사용을 기본으로 합니다. 
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

- **Preferred Language**: C++, Python, TypeScript
- **Editor**: vim
- **Communication Style**: 명확하고 간결하게, 필요한 경우 예시 제공

### Code style: Python
- PEP8 준수
- **MUST** yapf 포맷팅
- **MUST** google 스타일 가이드 준수

### Code style: C++
- **MUST** clangd, clang-tidy, clang-format 사용
- **MUST** google 스타일 가이드 준수

### Code style: TypeScript
- ESLint와 Prettier 사용
---

## 3. Global Tools & Skills (글로벌 도구 및 스킬)

어떤 프로젝트에서든 사용할 수 있는 전역 도구와 스킬입니다.

### 🛠️ Standard Tools
- `bash`: 터미널 명령어 실행
- `read`/`write`/`edit`: 파일 조작
- `glob`/`grep`: 파일 검색
- `webfetch`: 웹 콘텐츠 가져오기
- `todowrite`/`todoread`: 작업 목록 관리 및 조회 (Todo List)
---
