# Termux Agent Configuration (AGENTS.md)

이 파일은 **Termux** 안드로이드 터미널 환경 설정의 AI 에이전트 설정을 정의합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: Termux 안드로이드 터미널 환경 설정 관리
- **Goals**: 모바일 개발 환경 제공, 패키지 관리 자동화
- **Architecture**: 패키지 기반의 설정 구조
- **Domain Knowledge**: Termux, 안드로이드, 모바일 개발

### Tech Stack (기술 스택)
- **Language**: Shell Script
- **Framework**: Termux
- **Infrastructure**: Android

---

## 2. Folder Structure (폴더 구조)

```
[termux]/
├── setup.sh               # Termux 설정 스크립트
├── install_*.sh           # 각종 설치 스크립트
├── termux-ubuntu-setup    # Ubuntu 환경 설정
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: Shell Style Guide
- **Security**: 모바일 환경 특화의 보안 고려

---

## 4. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.