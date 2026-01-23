# WezTerm Agent Configuration (AGENTS.md)

이 파일은 **WezTerm** 설정 프로젝트의 AI 에이전트 설정을 정의합니다. 모던 터미널 에뮬레이터 설정 및 스크립트 관리에 대한 문맥, 구조, 규칙을 명시합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: WezTerm 터미널 에뮬레이터 설정 관리 시스템
- **Goals**: 고성능 터미널 환경 제공, 크로스플랫폼 지원, 설정 자동화
- **Architecture**: Lua 기반의 설정 파일 구조
- **Domain Knowledge**: WezTerm, Lua, 터미널 에뮬레이션, 크로스플랫폼 개발

### Tech Stack (기술 스택)
- **Language**: Lua
- **Framework**: WezTerm
- **Package Manager**: 시스템 패키지 매니저
- **Infrastructure**: Windows, macOS, Linux

---

## 2. Folder Structure (폴더 구조)

```
[wezterm]/
├── setup.sh               # WezTerm 설치 및 설정 스크립트
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Lua Style Guide (WezTerm 설정 규칙)
- **Naming Convention**: 
  - 설정 함수: camelCase (예: setupColors)
  - 변수: snake_case (예: config_options)
- **Formatting**: Lua 포매터 사용 권장
- **Linting**: WezTerm 설정 검증

### Security (보안)
- **Authentication**: N/A (로컬 터미널 설정)
- **Input Validation**: 모든 설정 옵션은 유효성 검증 필요
- **Secrets**: 민감한 경로나 설정은 환경 변수 사용
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.

---

## 4. Guardrails (가드레일)

- **Input Safety**: 설정 파일 주입 공격 방지를 위해 사용자 입력을 검증합니다.
- **Output Safety**: 터미널 설정이나 시스템 정보를 무분별하게 출력하지 않습니다.
- **Hallucination**: WezTerm 설정에 대한 사실이 아닌 정보를 생성하지 않도록 주의합니다.
- **Destructive Actions**: 시스템 전체 터미널 설정에 영향을 주는 작업은 신중하게 처리합니다.

---

## 5. Workflows (워크플로우)

### Configuration Workflow
1. **Requirement Analysis**: 필요한 WezTerm 기능 분석
2. **Setting Design**: 테마, 폰트, 키맵 설계
3. **Configuration**: wezterm.lua 파일 작성
4. **Testing**: WezTerm 재시작 및 기능 검증
5. **Documentation**: 설정 옵션 및 사용법 문서화

### Setup Workflow
1. **Environment Check**: 운영체제 및 WezTerm 버전 확인
2. **Installation**: WezTerm 설치 (필요시)
3. **Configuration**: 설정 파일 배포
4. **Verification**: 설정 로드 및 기능 확인

---

## 6. Environment Variables (환경 변수)

- `WEZTERM_CONFIG_DIR`: WezTerm 설정 디렉토리 경로
- `WEZTERM_VERSION`: WezTerm 버전
- `OS`: 운영체제 (windows, macos, linux)

---

## 7. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.