# Tailscale Agent Configuration (AGENTS.md)

이 파일은 **Tailscale** VPN 도구 설정의 AI 에이전트 설정을 정의합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: Tailscale VPN 도구 설정 관리
- **Goals**: 안전한 네트워크 연결 환경 제공
- **Architecture**: 서비스 기반의 설정 구조
- **Domain Knowledge**: Tailscale, VPN, 네트워크 보안

### Tech Stack (기술 스택)
- **Language**: Shell Script
- **Framework**: Tailscale
- **Infrastructure**: Linux/macOS/Windows (WSL)

---

## 2. Folder Structure (폴더 구조)

```
[tailscale]/
├── setup.sh               # Tailscale 설치 스크립트
├── install.sh             # Tailscale 설치 스크립트
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Shell Style Guide
- **Security**: VPN 인증 정보는 환경 변수로 관리

---

## 4. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.