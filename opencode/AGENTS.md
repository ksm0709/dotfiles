# OpenCode Agent Configuration (AGENTS.md)

이 파일은 **OpenCode** AI 개발 도구 모음의 AI 에이전트 설정을 정의합니다. 에이전트 시스템, 컨텍스트 관리, 설정 템플릿에 대한 문맥, 구조, 규칙을 명시합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: OpenCode AI 개발 도구 모음 및 에이전트 시스템
- **Goals**: AI 기반 개발 생산성 향상, 컨텍스트 관리, 에이전트 자동화
- **Architecture**: 모듈러 기반의 에이전트 시스템과 컨텍스트 메모리
- **Domain Knowledge**: AI 에이전트, 컨텍스트 관리, 메모리 시스템, 설정 자동화

### Tech Stack (기술 스택)
- **Language**: Python, Shell Script, TypeScript
- **Framework**: OpenMemory, Context Manager, MCP
- **Database**: SQLite (Context Memory)
- **Infrastructure**: Python 3.8+, Node.js, 다양한 운영체제

---

## 2. Folder Structure (폴더 구조)

```
[opencode]/
├── config/                 # 설정 파일 및 템플릿
│   ├── templates/         # 에이전트 템플릿
│   │   ├── project-agents-template.md
│   │   ├── agent-template.md
│   │   └── user-agents-template.md
│   ├── shared/            # 공유 설정
│   │   └── context/       # 컨텍스트 관리
│   │       ├── config.yaml
│   │       └── context_server.py
│   ├── agent/             # 에이전트 파일들
│   │   ├── agent-creator.md
│   │   ├── build.md
│   │   ├── cpp-review.md
│   │   ├── general.md
│   │   ├── plan.md
│   │   ├── pm.md
│   │   ├── py-code-reviewer.md
│   │   ├── research-analyst.md
│   │   └── senior-sw-engineer.md
│   └── custom-tools/      # 사용자 정의 도구 및 플러그인
│       ├── [plugin-name]/ # 개별 플러그인 디렉토리
│       │   ├── test/      # 플러그인 테스트 환경
│       │   ├── src/       # 플러그인 소스 코드
│       │   └── README.md  # 플러그인 문서
│       └── README.md      # custom-tools 가이드
├── install*.sh            # 설치 스크립트
├── setup.sh               # 메인 설정 스크립트
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: PEP 8 (Python), Shell Style Guide, TypeScript Standard
- **Naming Convention**: 
  - 파일: kebab-case (예: install-config.sh)
  - 변수: snake_case (예: context_manager)
  - 함수: snake_case (예: create_agent)
- **Formatting**: Black (Python), Prettier (TypeScript), ShellCheck
- **Linting**: pylint, eslint, shellcheck 사용

### Security (보안)
- **Authentication**: N/A (로컬 도구)
- **Input Validation**: 모든 사용자 입력과 에이전트 프롬프트는 검증되어야 합니다.
- **Secrets**: API 키나 개인정보는 환경 변수로 관리
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.

---

## 4. Guardrails (가드레일)

- **Input Safety**: 에이전트 프롬프트 주입 공격 방지를 위해 사용자 입력을 검증합니다.
- **Output Safety**: AI 응답이나 컨텍스트 데이터를 무분별하게 출력하지 않습니다.
- **Hallucination**: 에이전트 설정이나 동작에 대한 사실이 아닌 정보를 생성하지 않도록 주의합니다.
- **Destructive Actions**: 에이전트 시스템이나 컨텍스트 데이터에 영향을 주는 작업은 신중하게 처리합니다.

---

## 5. Workflows (워크플로우)

### Agent Creation Workflow
1. **Requirement Analysis**: 필요한 에이전트 역할 분석
2. **Template Selection**: 적절한 템플릿 선택
3. **Configuration**: 에이전트 설정 파일 작성
4. **Registration**: AGENTS.md에 에이전트 등록
5. **Testing**: 에이전트 동작 검증

### Context Management Workflow
1. **Session Init**: 컨텍스트 세션 초기화
2. **Memory Retrieval**: 관련 장기 기억 검색
3. **Working Memory**: 작업 중인 메모리 관리
4. **Checkpoint**: 주기적인 메모리 저장
5. **Session End**: 세션 종료 및 메모리 정리

### Setup Workflow
1. **Environment Check**: Python, Node.js 등 의존성 확인
2. **Installation**: 패키지 및 도구 설치
3. **Configuration**: 설정 파일 배포
4. **Initialization**: 컨텍스트 시스템 초기화
5. **Verification**: 시스템 동작 확인

### Plugin Development & Testing Workflow
1. **Create Test Directory**: 플러그인 테스트를 위한 격리된 환경 생성 (`test/` 디렉토리), opencode init
2. **Install in Test Environment**: 테스트 디렉토리에 플러그인 설치 및 기능 검증
3. **Stability Verification**: 플러그인의 안정성, 호환성, 오류 처리 검증
4. **Documentation**: README.md 작성 및 사용법 문서화
5. **Production Deployment**: 안정성 검증 완료 후에만 `$HOME/.config/opencode`에 설치

**⚠️ 중요**: 플러그인을 `$HOME/.config/opencode`에 직접 설치하지 마세요. 반드시 `custom-tools/[plugin-name]/test/` 디렉토리에서 먼저 테스트하고, 안정성이 검증된 후에만 프로덕션 환경에 배포하세요.

---

## 6. Environment Variables (환경 변수)

- `OPENSPEC_ROOT`: OpenSpec 루트 디렉토리
- `CONTEXT_MEMORY_PATH`: 컨텍스트 메모리 경로
- `AGENT_CONFIG_DIR`: 에이전트 설정 디렉토리
- `PYTHON_PATH`: Python 실행 경로
- `NODE_PATH`: Node.js 실행 경로

---

## 7. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.
