# Ralph Loop 결함 수정 재검증 Todo List

- [x] **0. Todo List 초기화**
  - [x] todowrite로 전체 작업 계획 수립
  - [x] 현재 단계를 in_progress로 설정

- [x] **1. 검증 준비**
  - [x] 오픈스펙 및 구현 코드 분석
  - [x] 테스트 환경 확인 (vitest 등)

- [x] **2. 스펙 추출**
  - [x] 관련 시나리오 추출 (retryCount 이관, 지시사항 중복 방지)

- [x] **3. 기능 테스트**
  - [x] `tests/ralph-loop.test.ts` 실행 및 결과 확인
  - [x] 필요 시 추가 테스트 케이스 작성 및 실행 (기존 테스트로 충분함 확인)

- [x] **4. 구현 검증**
  - [x] `src` 코드 내 retryCount 이관 로직 확인
  - [x] `src` 코드 내 지시사항 중복 방지 로직 확인

- [x] **5. 결과 리포트**
  - [ ] QA 검증 리포트 작성
  - [ ] PASS/FAIL 최종 판단
