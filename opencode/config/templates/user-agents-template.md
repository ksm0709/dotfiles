# Global Agent Configuration (AGENTS.md)

모든 프로젝트에 공통 적용되는 사용자(User) 레벨 글로벌 AI 에이전트 정책입니다.

## Quick Reference

- **Language (MUST)**: 대화/문서는 한국어 기본, 코드/주석은 영어 기본.
- **Safety (MUST)**: 파일 삭제, `git push -f` 등 파괴적 작업은 사용자 명시 승인 후 수행.
- **Validation (MUST)**: 코드 작성 후 실행 또는 테스트로 반드시 동작 검증.
- **Context (MUST)**: 전후 맥락 유지, 불필요한 중복 질문 최소화.
- **Skill Check (MUST)**: 작업 시작 전에 현재 태스크에 적합하고 유용한 스킬이 있는지 먼저 확인.
- **Testing (MUST)**: TDD 준수, 테스트는 프로젝트와 격리된 시스템 임시 디렉토리에서 수행.
- **Security (MUST)**: secret 하드코딩 금지, 환경 변수 사용, 입력 검증, 파라미터화된 쿼리 사용.
- **Preferred Stack**: C++, Python, TypeScript / Editor: `vim`.

## Detailed Instructions

- [Core Rules](.agent-guides/core-rules.md)
- [Security](.agent-guides/security.md)
- [Testing and Validation](.agent-guides/testing-and-validation.md)
- [Language and Style](.agent-guides/language-and-style.md)
- [Preferred Tools and Skills usage](.agent-guides/tools-and-skills.md)

## Scope and Priority

- 이 파일은 글로벌 기본 정책입니다.
- 프로젝트별 `AGENTS.md`가 존재하면 프로젝트 정책이 우선합니다.
- 프로젝트 정책에 없는 항목은 이 글로벌 정책을 따릅니다.
