# Tasks: Tasks Plugin Batch Operations & UI Enhancement

## Overview

이 문서는 tasks 플러그인 개선을 위한 구현 작업 목록입니다.

## Task List

### Phase 1: Core Implementation (Day 1-2)

#### 1.1 Type Definitions
- **ID**: 1.1
- **Title**: 확장된 타입 정의 추가
- **Description**: BatchOperation, BatchResult, CommandResultWithStatus 등 새로운 타입 정의
- **Dependencies**: None
- **Acceptance Criteria**:
  - [ ] `src/types/index.ts`에 새로운 타입 추가
  - [ ] 모든 타입에 적절한 JSDoc 주석
  - [ ] 기존 타입과의 호환성 확인

#### 1.2 Batch Command Implementation
- **ID**: 1.2
- **Title**: 배치 작업 명령어 구현
- **Description**: 여러 작업을 한 번에 처리하는 batch.ts 구현
- **Dependencies**: 1.1
- **Acceptance Criteria**:
  - [ ] `src/commands/batch.ts` 파일 생성
  - [ ] add/update/complete/remove 작업 지원
  - [ ] 부분적 실패 처리 (전체 롤백 없이 개별 결과 반환)
  - [ ] 최대 50개 작업 제한
  - [ ] 입력값 검증

#### 1.3 Enhanced Formatter
- **ID**: 1.3
- **Title**: 마크다운 포맷터 개선
- **Description**: 시각적 표현력을 높인 마크다운 포맷터 구현
- **Dependencies**: None
- **Acceptance Criteria**:
  - [ ] `src/lib/formatter.ts`에 새로운 포맷팅 함수 추가
  - [ ] 진행률 바 생성 함수
  - [ ] 작업 목록 테이블 형식 포맷팅
  - [ ] 이모지 아이콘 통합 (✅, 🔄, ⏳, 📋, 📊)
  - [ ] 상태별 섹션 구분 (완료/진행중/대기)

#### 1.4 Status Calculator
- **ID**: 1.4
- **Title**: 상태 요약 계산기
- **Description**: TaskList에서 통계 정보를 계산하는 유틸리티
- **Dependencies**: None
- **Acceptance Criteria**:
  - [ ] 완료/진행중/대기 작업 수 계산
  - [ ] 전체 진행률 계산 (0-100%)
  - [ ] StatusSummary 객체 반환

### Phase 2: Tool Integration (Day 2-3)

#### 2.1 Batch Tool Registration
- **ID**: 2.1
- **Title**: tasks_batch 툴 등록
- **Description**: Plugin의 tool 객체에 tasks_batch 추가
- **Dependencies**: 1.2, 1.3, 1.4
- **Acceptance Criteria**:
  - [ ] `src/index.ts`에 tasks_batch tool 정의
  - [ ] 적절한 description 및 args schema
  - [ ] sessionId 자동 추출
  - [ ] formattedOutput 반환

#### 2.2 Enhanced Existing Tools
- **ID**: 2.2
- **Title**: 기존 툴 응답 개선
- **Description**: 모든 기존 툴의 응답에 현황 정보 포함
- **Dependencies**: 1.3, 1.4
- **Acceptance Criteria**:
  - [ ] `src/commands/add-task.ts` 수정 - 현황 포함 응답
  - [ ] `src/commands/update.ts` 수정 - 현황 포함 응답
  - [ ] `src/commands/complete.ts` 수정 - 현황 포함 응답
  - [ ] `src/commands/remove.ts` 수정 - 현황 포함 응답
  - [ ] `src/commands/init.ts` 수정 - 현황 포함 응답
  - [ ] 각 툴의 execute 함수가 formattedOutput 반환

#### 2.3 Index.ts Updates
- **ID**: 2.3
- **Title**: 메인 인덱스 파일 업데이트
- **Description**: 모든 툴의 응답 형식 통일
- **Dependencies**: 2.1, 2.2
- **Acceptance Criteria**:
  - [ ] 모든 tool.execute가 formattedOutput 사용
  - [ ] 일관된 에러 처리 패턴
  - [ ] 응답에 이모지 및 마크다운 적용

### Phase 3: Testing (Day 3-4)

#### 3.1 Batch Command Tests
- **ID**: 3.1
- **Title**: 배치 명령어 테스트
- **Description**: batch.ts에 대한 단위 테스트
- **Dependencies**: 1.2
- **Acceptance Criteria**:
  - [ ] 여러 add 작업 테스트
  - [ ] 혼합 작업(add+update+complete) 테스트
  - [ ] 부분적 실패 처리 테스트
  - [ ] 최대 크기 제한 테스트
  - [ ] 잘못된 입력 검증 테스트

#### 3.2 Formatter Tests
- **ID**: 3.2
- **Title**: 포맷터 테스트
- **Description**: formatter.ts에 대한 단위 테스트
- **Dependencies**: 1.3
- **Acceptance Criteria**:
  - [ ] 진행률 바 포맷팅 테스트
  - [ ] 작업 목록 테이블 테스트
  - [ ] 배치 결과 포맷팅 테스트
  - [ ] 이모지 포함 여부 테스트
  - [ ] 마크다운 구조 검증

#### 3.3 Integration Tests
- **ID**: 3.3
- **Title**: 통합 테스트
- **Description**: 전체 플러그인 흐름 테스트
- **Dependencies**: 2.3
- **Acceptance Criteria**:
  - [ ] end-to-end batch 작업 테스트
  - [ ] 기존 툴과의 호환성 테스트
  - [ ] 응답 형식 일관성 테스트
  - [ ] 에러 시나리오 테스트

#### 3.4 Existing Tests Update
- **ID**: 3.4
- **Title**: 기존 테스트 업데이트
- **Description**: 변경된 응답 형식에 맞춰 기존 테스트 수정
- **Dependencies**: 2.2
- **Acceptance Criteria**:
  - [ ] `tests/commands/add-task.test.ts` 업데이트
  - [ ] `tests/commands/update.test.ts` 업데이트
  - [ ] `tests/commands/complete.test.ts` 업데이트
  - [ ] `tests/commands/remove.test.ts` 업데이트
  - [ ] 모든 기존 테스트 통과

### Phase 4: Documentation (Day 4)

#### 4.1 README Update
- **ID**: 4.1
- **Title**: README 문서 업데이트
- **Description**: 새로운 기능에 대한 문서화
- **Dependencies**: 2.3
- **Acceptance Criteria**:
  - [ ] tasks_batch 사용법 문서화
  - [ ] 배치 작업 예시 코드 추가
  - [ ] 새로운 응답 형식 설명
  - [ ] 마이그레이션 가이드 추가

#### 4.2 API Documentation
- **ID**: 4.2
- **Title**: API 문서 작성
- **Description**: 새로운 툴의 API 명세
- **Dependencies**: 2.1
- **Acceptance Criteria**:
  - [ ] tasks_batch 파라미터 문서화
  - [ ] BatchOperation 타입 설명
  - [ ] 응답 형식 예시
  - [ ] 에러 처리 설명

### Phase 5: Review & QA (Day 5)

#### 5.1 Code Review
- **ID**: 5.1
- **Title**: 코드 리뷰
- **Description**: 구현된 코드 검토
- **Dependencies**: 3.4
- **Acceptance Criteria**:
  - [ ] TypeScript 타입 안전성 확인
  - [ ] 에러 처리 패턴 검토
  - [ ] 코드 중복 제거 확인
  - [ ] 성능 최적화 검토

#### 5.2 QA Verification
- **ID**: 5.2
- **Title**: QA 검증
- **Description**: 기능적 요구사항 충족 여부 확인
- **Dependencies**: 3.3, 4.1
- **Acceptance Criteria**:
  - [ ] 배치 작업이 5개 이상 한 번에 처리되는지 확인
  - [ ] 모든 작업 후 현황이 자동 표시되는지 확인
  - [ ] 마크다운 UI가 정상 렌더링되는지 확인
  - [ ] 기존 툴과의 하위 호환성 확인

#### 5.3 Performance Test
- **ID**: 5.3
- **Title**: 성능 테스트
- **Description**: 배치 작업 성능 검증
- **Dependencies**: 3.1
- **Acceptance Criteria**:
  - [ ] 배치 작업이 개별 호출보다 20% 이상 빠른지 확인
  - [ ] 대용량 task list(100개+)에서 응답 시간 측정
  - [ ] 메모리 사용량 확인

## Dependencies Graph

```
1.1 (Types)
  ↓
1.2 (Batch Command) ← 1.4 (Status Calculator)
  ↓                    ↑
2.1 (Batch Tool) ← 1.3 (Formatter)
  ↓
2.2 (Enhanced Tools) ← 1.3, 1.4
  ↓
2.3 (Index Updates)
  ↓
3.1 (Batch Tests) ← 1.2
3.2 (Formatter Tests) ← 1.3
3.3 (Integration Tests) ← 2.3
3.4 (Existing Tests Update) ← 2.2
  ↓
4.1 (README) ← 2.3
4.2 (API Docs) ← 2.1
  ↓
5.1 (Code Review) ← 3.4
5.2 (QA) ← 3.3, 4.1
5.3 (Performance) ← 3.1
```

## Risk Mitigation

| Risk | Task IDs | Mitigation |
|------|----------|------------|
| 기존 테스트 깨짐 | 3.4 | 기존 테스트를 먼저 확인하고 점진적으로 업데이트 |
| 응답 크기 증가 | 1.3 | 큰 task list에서는 요약 정보만 포함하는 옵션 추가 |
| 배치 작업 실패 | 1.2 | 트랜잭션 패턴 적용, 실패한 작업 명확히 표시 |
| 타입 충돌 | 1.1 | 기존 타입을 확장하는 방식으로 구현 |

## Definition of Done

- [ ] 모든 Phase 1-5 작업 완료
- [ ] 모든 테스트 통과 (커버리지 80% 이상)
- [ ] 코드 리뷰 완료
- [ ] QA 검증 완료
- [ ] 문서화 완료
- [ ] 기존 기능과의 하위 호환성 확인
- [ ] 성능 기준 충족 (배치 작업 20% 이상 효율적)
