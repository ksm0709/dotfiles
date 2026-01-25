# Tasks: Ralph Loop Plugin Implementation

## Phase 1: 환경 설정 및 기본 구조 (Setup)
- [ ] 플러그인 프로젝트 초기화 및 의존성 설치 (`@opencode-ai/plugin`, `@opencode-ai/sdk`)
- [ ] 기본 플러그인 스켈레톤 코드 작성 (`index.ts`)
- [ ] 설정 로드 로직 구현 (`Hooks.config`)

## Phase 2: 스펙 주도 테스트 개발 (Spec-Driven Test Dev)
- [ ] Vitest/Jest 테스트 환경 구성 및 Mock SDK 설정
- [ ] `proposal.md`의 시나리오별 테스트 케이스 작성 (Red 단계)
  - [ ] 최초 프롬프트 인젝션 테스트
  - [ ] Promise Word 감지 및 루프 종료 테스트
  - [ ] Promise Word 부재 시 루프 실행 및 요약 저장 테스트
  - [ ] 최대 반복 횟수 초과 시 중단 테스트

## Phase 3: 핵심 로직 구현 (Core Logic)
- [ ] `chat.message` 훅을 이용한 첫 번째 사용자 메시지 가로채기 및 지시사항(Promise Word) 주입 로직 구현
- [ ] `session.idle` 이벤트 핸들러 구현 및 `LoopManager` 연동
- [ ] `client.session.messages`를 통한 마지막 메시지 추출 및 Promise Word 포함 여부 검증 로직 구현
- [ ] `client.session.summarize` 호출 및 `.opencode/sessions/(session-id)/ralph_summary.md` 경로에 요약 파일 저장 로직 구현 (디렉토리 자동 생성 포함)
- [ ] 세션 초기화(`delete` & `create`) 및 재시작(`prompt`) 로직 구현 (재시작 프롬프트에 Promise Word 지시사항 재포함)

## Phase 4: 제어 및 예외 처리 (Control & Error Handling)
- [ ] 세션별 반복 횟수 관리 로직 구현
- [ ] 최대 반복 횟수 도달 시 알림 처리
- [ ] API 호출 실패 시 재시도 및 로깅 전략 수립

## Phase 5: 검증 및 문서화 (Verification)
- [ ] 전체 테스트 케이스 통과 확인 (Green 단계)
- [ ] 실제 OpenCode 환경에서 통합 테스트 수행
- [ ] 사용자 가이드 작성 (README.md)
