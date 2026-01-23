# Tmux Agent Configuration (AGENTS.md)

이 파일은 **Tmux** 설정 프로젝트의 AI 에이전트 설정을 정의합니다. 터미널 멀티플렉서 설정 및 스크립트 관리에 대한 문맥, 구조, 규칙을 명시합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: Tmux 터미널 멀티플렉서 설정 관리 시스템
- **Goals**: 효율적인 터미널 환경 제공, 세션 관리 자동화, 생산성 향상
- **Architecture**: 단일 설정 파일(tmux.conf) 기반의 중앙화된 구조
- **Domain Knowledge**: Tmux, 셸 스크립트, 터미널 환경, 키맵 설정

### Tech Stack (기술 스택)
- **Language**: Shell Script, Tmux Configuration
- **Framework**: Tmux 3.0+
- **Package Manager**: 시스템 패키지 매니저 (apt, brew, etc.)
- **Infrastructure**: Linux/macOS/Windows (WSL)

---

## 2. Folder Structure (폴더 구조)

```
[tmux]/
├── setup.sh               # Tmux 설치 및 설정 스크립트
├── tmux.conf              # Tmux 설정 파일
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Tmux 설정 가이드라인
- **Naming Convention**: 
  - 설정 옵션: tmux 표준 명명 규칙 따름
  - 키맵: 직관적인 조합 사용 (예: Ctrl-a, Ctrl-b)
- **Formatting**: 설정 옵션 그룹화 및 주석
- **Linting**: Tmux 설정 검증 도구 사용

### Security (보안)
- **Authentication**: N/A (로컬 터미널 설정)
- **Input Validation**: 모든 설정 옵션은 유효성 검증 필요
- **Secrets**: 민감한 명령어나 경로는 설정에 직접 포함하지 않음
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.

---

## 4. Guardrails (가드레일)

- **Input Safety**: 설정 파일 주입 공격 방지를 위해 사용자 입력을 검증합니다.
- **Output Safety**: 터미널 세션 정보나 민감한 데이터를 출력하지 않습니다.
- **Hallucination**: Tmux 설정에 대한 사실이 아닌 정보를 생성하지 않도록 주의합니다.
- **Destructive Actions**: 실행 중인 세션에 영향을 주는 작업은 신중하게 처리합니다.

---

## 5. Workflows (워크플로우)

### Configuration Workflow
1. **Requirement Analysis**: 필요한 Tmux 기능 분석
2. **Setting Design**: 키맵 및 옵션 설계
3. **Configuration**: tmux.conf 파일 작성
4. **Testing**: Tmux 재시작 및 기능 검증
5. **Documentation**: 사용법 및 단축키 문서화

### Setup Workflow
1. **Environment Check**: 시스템 및 Tmux 버전 확인
2. **Installation**: Tmux 설치 (필요시)
3. **Configuration**: 설정 파일 배포 및 심볼릭 링크 생성
4. **Verification**: 설정 로드 및 기능 확인

---

## 6. Environment Variables (환경 변수)

- `TMUX_CONFIG_DIR`: Tmux 설정 디렉토리 경로
- `TMUX_VERSION`: Tmux 버전
- `SHELL`: 현재 사용 중인 셸

---

## 7. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.