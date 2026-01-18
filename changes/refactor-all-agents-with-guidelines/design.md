# Design: 에이전트 가이드라인 섹션 통합 설계

## 1. 섹션 재구성 전략

### Guidelines > Boundary
- **Must**: 에이전트가 반드시 지켜야 할 핵심 원칙 (예: 한국어 사용, 테스트 작성).
- **Never**: 금지 사항 (예: 비밀번호 커밋 금지, 무단 푸시 금지).

### Guidelines > Commands & Skills
- **Preferred Tools & Skills**: 에이전트가 주력으로 사용하는 도구(bash, git 등)와 스킬(deep-research 등).
- **Restricted Commands & Skills**: 사용 시 주의가 필요한 도구.

### Guidelines > Conventions
- **Code Style**: 언어별 스타일 가이드.
- **Documentation**: 문서 작성 표준.

## 2. 에이전트별 적용 포인트

- **Senior SW Engineer**: `Testing Strategy`, `Anti-Patterns` 등을 `Guidelines`로 이동.
- **Py Code Reviewer**: `Review Focus Areas`, `Feedback Style` 등을 `Guidelines`로 이동.
- **Research Analyst**: `스킬 활용 가이드`를 `Commands & Skills`로 이동.
- **PM**: `의사결정 및 품질 프레임워크`를 `Guidelines`로 이동.

## ADDED Requirements
- 모든 에이전트 파일은 `## 가이드라인 (Guidelines)` 섹션을 포함해야 한다.
- `Commands` 섹션은 `Commands & Skills`로 명칭을 변경한다.
- `agent-template.md`의 최신 Mermaid 그래프(`Plan with Todo`)를 적용한다.
