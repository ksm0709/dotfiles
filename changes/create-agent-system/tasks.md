# Tasks: 에이전트 시스템 구현

## 1. 템플릿 생성
- [ ] `opencode/config/agent/agent-template.md` 파일 작성
    - `pm.md` 내용 참조하여 일반화
    - 워크플로우 다이어그램 포함

## 2. 에이전트 생성기 구현
- [ ] `opencode/config/agent/agent-creator.md` 파일 작성
    - 역할: 템플릿 기반 에이전트 생성
    - 프롬프트 엔지니어링: 템플릿 준수 강제

## 3. 리서치 에이전트 구현
- [ ] `opencode/config/agent/research-analyst.md` 파일 작성
    - 스킬: `deep-research`, `research`
    - 도구: `webfetch`, `read`, `write` 등 기본 도구
    - 모델: `opencode/big-pickle` (또는 고성능 모델)

## 4. 검증
- [ ] `openspec validate create-agent-system` 실행
- [ ] 생성된 파일들의 문법 및 구조 확인
