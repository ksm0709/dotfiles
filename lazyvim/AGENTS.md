# LazyVim Agent Configuration (AGENTS.md)

이 파일은 **LazyVim** 설정 프로젝트의 AI 에이전트 설정을 정의합니다. Neovim 설정 및 LazyVim 플러그인 관리에 대한 문맥, 구조, 규칙을 명시합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: LazyVim 기반의 Neovim 설정 관리 시스템
- **Goals**: 일관된 Neovim 환경 제공, 플러그인 자동화, 개발 생산성 향상
- **Architecture**: LazyVim 플러그인 매니저 기반의 모듈러 설정 구조
- **Domain Knowledge**: Neovim, Lua, LazyVim, 플러그인 개발, Vim 스크립트

### Tech Stack (기술 스택)
- **Language**: Lua, Vim Script
- **Framework**: LazyVim, Neovim
- **Package Manager**: Lazy.nvim
- **Infrastructure**: Neovim 0.9+, 다양한 운영체제

---

## 2. Folder Structure (폴더 구조)

```
[lazyvim]/
├── nvv/                    # 기본 Neovim 설정
│   ├── init.lua           # 메인 설정 파일
│   ├── lua/               # Lua 설정 모듈
│   │   ├── config/        # 설정 옵션
│   │   ├── plugins/       # 플러그인 설정
│   │   └── keymaps/       # 키맵 설정
│   └── stylua.toml        # Lua 포매터 설정
├── nvv-termux/            # Termux용 Neovim 설정
│   ├── init.lua           # Termux 메인 설정
│   └── lua/               # Termux 전용 설정
├── setup.sh               # LazyVim 설치 스크립트
├── install-*.sh           # 각종 설치 스크립트
├── backup-*.sh            # 백업 스크립트
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Lua Style Guide (Stylua 기반)
- **Naming Convention**: 
  - 파일: kebab-case (예: keymaps.lua)
  - 변수: snake_case (예: config_options)
  - 함수: snake_case (예: setup_plugins)
- **Formatting**: Stylua 사용
- **Linting**: luacheck 사용 권장

### Security (보안)
- **Authentication**: N/A (로컬 에디터 설정)
- **Input Validation**: 모든 사용자 설정은 검증되어야 합니다.
- **Secrets**: API 키나 개인정보는 설정에 하드코딩하지 않고 환경 변수 사용
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.

---

## 4. Guardrails (가드레일)

- **Input Safety**: 설정 파일 주입 공격 방지를 위해 사용자 입력을 검증합니다.
- **Output Safety**: 개인정보나 민감한 설정 정보를 출력하지 않습니다.
- **Hallucination**: Neovim 설정에 대한 사실이 아닌 정보를 생성하지 않도록 주의합니다.
- **Destructive Actions**: 기존 설정 파일 삭제 시 반드시 백업 후 사용자 승인을 받습니다.

---

## 5. Workflows (워크플로우)

### Plugin Management Workflow
1. **Requirement Analysis**: 필요한 플러그인 기능 분석
2. **Plugin Selection**: LazyVim 호환 플러그인 선택
3. **Configuration**: 플러그인 설정 작성 (lua/plugins/*.lua)
4. **Testing**: Neovim 재시작 및 기능 검증
5. **Documentation**: 사용법 및 키맵 문서화

### Setup Workflow
1. **Environment Check**: Neovim 버전 및 의존성 확인
2. **Backup**: 기존 nvim 설정 백업
3. **Installation**: LazyVim 설치 및 설정 복사
4. **Configuration**: 개인 설정 적용
5. **Verification**: 플러그인 로드 및 기능 확인

---

## 6. Environment Variables (환경 변수)

- `NVIM_CONFIG_DIR`: Neovim 설정 디렉토리 경로
- `LAZYVIM_VERSION`: LazyVim 버전
- `TERMUX_ENV`: Termux 환경 여부 (true/false)

---

## 7. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.