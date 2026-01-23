<!-- OPENSPEC:START -->
# OpenSpec Instructions

These instructions are for AI assistants working in this project.

Always open `@/openspec/AGENTS.md` when the request:
- Mentions planning or proposals (words like proposal, spec, change, plan)
- Introduces new capabilities, breaking changes, architecture shifts, or big performance/security work
- Sounds ambiguous and you need the authoritative spec before coding

Use `@/openspec/AGENTS.md` to learn:
- How to create and apply change proposals
- Spec format and conventions
- Project structure and guidelines

Keep this managed block so 'openspec update' can refresh the instructions.

<!-- OPENSPEC:END -->

# Dotfiles Agent Configuration (AGENTS.md)

이 파일은 **Dotfiles** 프로젝트의 AI 에이전트 설정을 정의합니다. 프로젝트의 문맥, 구조, 규칙, 워크플로우를 명시하여 에이전트가 일관되고 안전하게 작업을 수행하도록 돕니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: 개인 개발 환경 설정 스크립트 및 dotfiles 관리 시스템
- **Goals**: 다양한 개발 환경에서 일관된 설정을 제공하고 자동화된 설치/관리 기능 제공
- **Architecture**: 모듈러 기반의 설정 스크립트 구조 (각 도구별 독립적인 setup.sh)
- **Domain Knowledge**: Shell 스크립트, 개발 환경 설정, 패키지 관리, dotfiles 관리

### Tech Stack (기술 스택)
- **Language**: Bash, Shell Script, Python (Context Server)
- **Framework**: N/A (순수 스크립트 기반)
- **Database**: SQLite (Context Memory)
- **Infrastructure**: Linux/macOS/Windows (WSL), 다양한 터미널 환경

---

## 2. Folder Structure (폴더 구조)

이 프로젝트의 디렉토리 구조와 각 폴더의 역할입니다. 에이전트는 코드를 생성하거나 수정할 때 이 구조를 준수해야 합니다.

```
[Project Root]/
├── opencode/                 # OpenCode 설정 및 도구
│   ├── config/              # 설정 파일 및 템플릿
│   │   ├── templates/       # 에이전트 템플릿
│   │   ├── shared/          # 공유 설정
│   │   └── agent/           # 프로젝트 전용 에이전트
│   ├── install*.sh          # 설치 스크립트
│   └── setup.sh             # 메인 설정 스크립트
├── changes/                  # 변경 제안 및 설계 문서
│   └── [change-name]/       # 개별 변경 항목
│       ├── proposal.md      # 제안서
│       ├── design.md        # 설계 문서
│       └── tasks.md         # 작업 목록
├── lazyvim/                  # LazyVim 설정
├── tmux/                     # Tmux 설정
├── wezterm/                  # WezTerm 설정
├── [tool-name]/             # 기타 도구별 설정 폴더
│   ├── setup.sh             # 도구 설치 스크립트
│   └── [config-files]       # 도구 설정 파일
├── AGENTS.md                 # 이 파일
└── README.md                 # 프로젝트 설명
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Shell Style Guide (Google Shell Style Guide 기반)
- **Naming Convention**: 
  - 파일: kebab-case (예: install-config.sh)
  - 변수: snake_case (예: config_path)
  - 함수: snake_case (예: install_dependencies)
- **Formatting**: ShellCheck 기반의 린팅
- **Linting**: ShellCheck 사용 권장

### Security (보안)
- **Authentication**: N/A (로컬 설정 스크립트)
- **Input Validation**: 모든 사용자 입력과 매개변수는 검증되어야 합니다.
- **Secrets**: API 키나 비밀번호는 코드에 하드코딩하지 않고 환경 변수를 사용합니다.
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.
- **Parameterized queries only**: SQL 인젝션 방지를 위해 반드시 파라미터화된 쿼리를 사용합니다.

---

## 4. Guardrails (가드레일)

에이전트가 반드시 지켜야 할 안전 수칙과 제한 사항입니다.

- **Input Safety**: 프롬프트 인젝션 공격을 방지하기 위해 사용자 입력을 검증합니다.
- **Output Safety**: 개인정보(PII), 독성 콘텐츠, 보안에 민감한 정보를 출력하지 않습니다.
- **Hallucination**: 사실이 아닌 정보를 생성하지 않도록 주의하며, 불확실한 경우 사용자에게 확인합니다.
- **Destructive Actions**: 파일 삭제, 강제 푸시 등 파괴적인 작업은 사용자의 명시적 승인 없이 수행하지 않습니다.
- **System Safety**: 시스템 전체 설정을 변경하는 작업은 신중하게 처리하고 사용자에게 명확히 알립니다.

---

## 5. Workflows (워크플로우)

### Development Workflow
1. **Task Analysis**: 요구사항 분석 및 계획 수립
2. **Implementation**: 스크립트 작성 (테스트 케이스 포함)
3. **Testing**: 로컬 환경에서 스크립트 실행 및 검증
4. **Review**: 코드 리뷰 및 리팩토링

### Setup Workflow
1. **Environment Check**: 대상 환경 및 의존성 확인
2. **Backup**: 기존 설정 백업
3. **Installation**: 패키지 및 도구 설치
4. **Configuration**: 설정 파일 배포 및 심볼릭 링크 생성
5. **Verification**: 설치 결과 확인

### Change Proposal Workflow
1. **Proposal**: changes/ 디렉토리에 제안서 작성
2. **Design**: 상세 설계 문서 작성
3. **Tasks**: 구현 작업 목록 작성
4. **Implementation**: 작업 수행 및 검증
5. **Review**: 결과 검토 및 반영

---

## 6. Environment Variables (환경 변수)

프로젝트 실행에 필요한 환경 변수 목록입니다. (`.env` 파일 관리)

- `PROJECT_ROOT`: 프로젝트 루트 디렉토리 경로
- `SHELL`: 현재 사용 중인 셸 (bash, zsh, fish 등)
- `OS`: 운영체제 (linux, macos, windows)
- `INSTALL_MODE`: 설치 모드 (interactive, silent)
- `BACKUP_DIR`: 설정 백업 디렉토리 경로

---

## 7. Agent Registry (에이전트 명부)

이 프로젝트에서 활용 가능한 전문 에이전트 목록입니다. (글로벌 에이전트 외 프로젝트 전용 에이전트만 기술)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.