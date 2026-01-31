# Proposal: Tasks Plugin Batch Operations & UI Enhancement

## Change ID
`tasks-plugin-batch-operations`

## Status
PROPOSED

## Related Changes
- None

## Summary
Tasks 플러그인의 사용자 경험을 개선하기 위해 세 가지 핵심 문제를 해결합니다:
1. **툴 호출 단순화**: 개별 툴 호출 대신 한 번에 여러 작업을 처리하는 배치(batch) 작업 지원
2. **작업 현황 가시성**: 작업 업데이트 후 즉시 전체 task 현황을 표시
3. **UI/UX 개선**: 마크다운 기반의 체크리스트 스타일로 task 현황을 시각적으로 표현

## Motivation

### 현재 문제점

#### 1. 과도한 툴 호출 (Too Many Tool Calls)
현재 tasks 플러그인은 각 작업(add, update, remove, complete)을 별개의 툴로 제공합니다. 에이전트가 여러 작업을 수행하려면 각각의 툴을 개별적으로 호출해야 합니다:

```
[에이전트] → tasks_add("작업1") → [플러그인]
[에이전트] → tasks_add("작업2") → [플러그인]
[에이전트] → tasks_update("1", "in_progress") → [플러그인]
[에이전트] → tasks_complete("1") → [플러그인]
```

이는:
- **비효율적인 통신**: 여러 번의 왕복(round-trip) 발생
- **컨텍스트 전환 비용**: 각 툴 호출 간 상태 관리의 복잡성
- **에이전트 부담**: 여러 개의 소규모 툴 호출을 관리해야 하는 부담

#### 2. 작업 현황 파악의 어려움
작업을 추가/수정/삭제한 후에도 현재 전체 task 현황을 보려면 별도로 `tasks_list`나 `tasks_status`를 호출해야 합니다. 이는:
- **피드백 지연**: 작업 결과를 즉시 확인할 수 없음
- **추가 툴 호출 필요**: 현황 파악을 위한 추가 작업 필요
- **사용자 경험 저하**: 작업 흐름의 연속성이 끊김

#### 3. UI/UX 개선 필요성
현재는 텍스트 기반의 단순한 응답만 제공됩니다. todowrite/todoread처럼 시각적으로 체크리스트가 표시되면:
- **가독성 향상**: 작업 상태를 한눈에 파악 가능
- **직관적인 표현**: 체크박스, 진행률 등 시각적 요소 제공
- **일관된 경험**: 기존 todo 도구와 유사한 UX 제공

## Proposed Solution

### 1. 배치 작업 지원 (Batch Operations)

새로운 `tasks_batch` 툴을 도입하여 한 번의 호출로 여러 작업을 처리합니다:

```typescript
tasks_batch({
  operations: [
    { type: 'add', title: '작업1', parent: '1' },
    { type: 'add', title: '작업2' },
    { type: 'update', id: '1', status: 'in_progress' },
    { type: 'complete', id: '2' },
    { type: 'remove', id: '3' }
  ]
})
```

**장점**:
- 단일 툴 호출로 여러 작업 수행
- 트랜잭션 단위로 작업 관리 가능
- 네트워크/통신 오버헤드 감소

### 2. 작업 후 현황 자동 표시

모든 작업 툴(init, add, update, remove, complete, batch)의 응답에 현재 전체 task 현황을 포함합니다:

```markdown
## ✅ 작업 완료
Task "작업1"이 추가되었습니다. (ID: 4)

## 📋 현재 작업 현황

### 진행 상황 요약
- ✅ 완료: 2개
- 🔄 진행중: 1개
- ⏳ 대기: 3개
- 📊 전체 진행률: 33%

### 작업 목록
| ID | 작업 | 상태 | 상세 |
|----|------|------|------|
| 1 | 요구사항 분석 | ✅ 완료 | - |
| 2 | 설계 문서 작성 | 🔄 진행중 | 50% 완료 |
| 3 | 구현 | ⏳ 대기 | - |
| 4 | 작업1 | ⏳ 대기 | 방금 추가 |
```

### 3. 마크다운 기반 UI 개선

모든 응답을 마크다운 형식으로 구조화하여 시각적 표현력을 높입니다:

- **이모지 아이콘**: ✅, 🔄, ⏳, 📋, 📊, ⚠️, ❌ 등으로 상태 구분
- **체크리스트**: `- [x]` 형태로 완료/미완료 표시
- **테이블**: 작업 목록을 테이블 형태로 정리
- **헤더 구분**: `##`, `###`로 섹션 구분
- **진행률 바**: 텍스트 기반 진행률 표시 (`[████░░░░░░] 40%`)

## Success Criteria

### 기능적 요구사항
1. **배치 작업**: 단일 `tasks_batch` 툴 호출로 최소 5개 이상의 작업을 한 번에 처리
2. **현황 표시**: 모든 작업 툴의 응답에 현재 task 현황 요약이 자동으로 포함
3. **UI 개선**: 마크다운 기반의 시각적 표현으로 가독성 향상 (이모지, 테이블, 체크리스트 활용)

### 비기능적 요구사항
1. **하위 호환성**: 기존 개별 툴(tasks_add, tasks_update 등)은 그대로 유지
2. **성능**: 배치 작업 처리 시간이 개별 호출의 합보다 20% 이상 빨라야 함
3. **에러 처리**: 배치 작업 중 일부 실패 시 성공/실패 작업을 명확히 구분하여 표시

## Impact Analysis

### Affected Components
- `tasks` 플러그인 전체
- `src/index.ts`: 새로운 `tasks_batch` 툴 추가
- `src/commands/batch.ts`: 배치 작업 처리 로직 신규
- `src/lib/formatter.ts`: 마크다운 포맷팅 개선
- 모든 command 파일: 응답에 현황 정보 포함하도록 수정

### Breaking Changes
**없음** - 기존 툴들은 그대로 유지되며, 새로운 기능은 추가 형태로 제공

### Dependencies
- `@opencode-ai/plugin`: 기존 버전과 동일
- 추가 의존성 없음

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| 배치 작업 중간 실패 | 중간 상태에서 롤백 필요 | 트랜잭션 패턴 적용, 실패 시 이전 작업 롤백 |
| 응답 크기 증가 | 큰 task list에서 응답이 너무 커짐 | 요약 정보만 포함, 상세는 옵션으로 제공 |
| 마크다운 파싱 오류 | 특수 문자 처리 문제 | 모든 출력을 마크다운 escape 처리 |

## Timeline

| Phase | Duration | Description |
|-------|----------|-------------|
| Design | 1 day | 상세 설계 및 스펙 확정 |
| Implementation | 2 days | 배치 작업 및 UI 개선 구현 |
| Testing | 1 day | 단위 테스트 및 통합 테스트 |
| Review | 1 day | 코드 리뷰 및 QA |
| **Total** | **5 days** | |

## Open Questions

1. 배치 작업의 최대 크기 제한이 필요한가? (예: 한 번에 50개 작업으로 제한)
2. 현황 표시에서 전체 목록을 모두 보여줄 것인가, 아니면 요약만 보여줄 것인가?
3. 기존 툴들을 deprecated로 표시하고 점진적으로 제거할 것인가, 아니면 영구히 유지할 것인가?

## Appendix

### 참고: 기존 툴 구조

```typescript
// 현재 구조 - 개별 툴 호출
tasks_init({ agent, title })
tasks_add({ title, parent })
tasks_update({ id, status })
tasks_complete({ id })
tasks_remove({ id })
tasks_list({ format })
tasks_status({})

// 제안 구조 - 배치 툴 추가
tasks_batch({ operations: [...] })  // NEW
```

### 참고: 마크다운 UI 예시

```markdown
## 📋 Task Status: API-구현

### 🎯 진행 상황
- ✅ 완료: 3개 (37.5%)
- 🔄 진행중: 2개 (25%)
- ⏳ 대기: 3개 (37.5%)

### 📊 전체 진행률
[██████░░░░░░░░░░] 62.5%

### 📝 작업 목록
#### ✅ 완료된 작업
- [x] 1. 요구사항 분석
- [x] 2. 설계 문서 작성
- [x] 3. 테스트 코드 작성

#### 🔄 진행중인 작업
- [ ] 4. API 엔드포인트 구현 (50%)
- [ ] 5. 데이터베이스 연동 (30%)

#### ⏳ 대기중인 작업
- [ ] 6. 인증 미들웨어 개발
- [ ] 7. 에러 핸들링 구현
- [ ] 8. 문서화
```
