# Gemini CLI Agent Configuration (AGENTS.md)

이 파일은 **Gemini CLI** AI 도구 설정의 AI 에이전트 설정을 정의합니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: Gemini CLI AI 도구 설정 관리
- **Goals**: AI 기반 명령줄 도구 환경 제공
- **Architecture**: 설정 파일 기반의 도구 관리
- **Domain Knowledge**: Gemini API, CLI 도구, AI 통합

### Tech Stack (기술 스택)
- **Language**: JSON, Shell Script
- **Framework**: Gemini API
- **Infrastructure**: Node.js, 다양한 운영체제

---

## 2. Folder Structure (폴더 구조)

```
[gemini-cli]/
├── settings.json          # Gemini CLI 설정 파일
├── setup.sh               # 설치 스크립트
├── install-*.sh           # 각종 설치 스크립트
└── AGENTS.md              # 이 파일
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: JSON 표준, Shell Style Guide
- **Security**: API 키는 환경 변수로 관리

---

## 4. Agent Registry (에이전트 명부)

현재 프로젝트 전용 에이전트가 없습니다. 글로벌 에이전트를 사용하세요.