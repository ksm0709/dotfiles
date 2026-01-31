# Proposal: Tasks Plugin Unification & Session Isolation

## Change ID
`unify-tasks-tool-and-session-isolation`

## Status
PROPOSED

## Summary
Tasks 플러그인의 7개 개별 툴(`tasks_init`, `tasks_add`, `tasks_remove`, `tasks_update`, `tasks_complete`, `tasks_list`, `tasks_status`)을 **단일 툴 `tasks`로 통합**하고, **세션 격리(Session Isolation)**를 강화하여 현재 세션의 작업만 표시하도록 개선합니다.

## Motivation

### 현재 문제점
1. **툴 인터페이스 복잡성**: 7개의 개별 툴을 학습하고 사용해야 함
2. **일관성 부족**: 각 툴의 사용 패턴이 달라 사용자 경험이 단절됨
3. **세션 간 데이터 노출**: `tasks_list`, `tasks_status`가 모든 세션의 작업을 표시하여 프라이버시/보안 문제 발생
4. **배치 작업 미지원**: `tasks_batch` 툴이 코드에는 존재하나 실제로는 사용 불가

### 개선 목표
1. **단순화**: 단일 `tasks` 툴로 모든 작업 관리
2. **세션 격리**: 현재 세션의 작업만 표시 (보안/프라이버시 강화)
3. **배치 작업 지원**: 여러 작업을 한 번에 처리
4. **일관된 출력**: 모든 작업 후 현재 세션의 상태 자동 표시

## Detailed Design

### 1. 툴 인터페이스 변경

#### AS-IS (현재)
```typescript
// 7개 개별 툴
tasks_init(agent: string, title: string)
tasks_add(title: string, parent?: string)
tasks_update(id: string, status: 'pending' | 'in_progress' | 'completed')
tasks_complete(id: string)
tasks_remove(id: string)
tasks_list(format?: 'markdown' | 'json' | 'table')
tasks_status()
```

#### TO-BE (개선)
```typescript
// 단일 통합 툴
tasks(operations: Operation[])

interface Operation {
  type: 'init' | 'add' | 'update' | 'complete' | 'remove';
  // type별 선택적 필드
  agent?: string;           // for init
  title?: string;           // for init, add
  id?: string;              // for update, complete, remove
  parent?: string;          // for add
  status?: TaskStatus;      // for update
}
```

### 2. 세션 격리 (Session Isolation)

#### AS-IS
- `tasks_list`, `tasks_status`가 **모든 세션**의 작업 목록 표시
- 세션 간 데이터가 노출됨

#### TO-BE
- **현재 세션의 작업만** 표시
- 다른 세션의 작업은 완전히 격리
- 세션 ID는 OpenCode 컨텍스트에서 자동 추출

### 3. 출력 형식

#### AS-IS
- 툴별로 다른 출력 형식
- `tasks_list`: 전체 세션의 모든 작업 목록
- `tasks_status`: 전체 세션의 상태 요약

#### TO-BE
- **통일된 출력**: 현재 세션의 작업 목록 + 상태 요약
- 항상 마크다운 형식으로 표시
- 작업 완료 후 자동으로 현재 상태 표시

```markdown
# Task List: {title}

**에이전트**: {agent}
**세션 ID**: {sessionId} (현재 세션만)

## 작업 목록
- [x] ✅ **1**. 작업 제목
- [ ] ⏳ **2**. 작업 제목
  - [x] ✅ **2.1**. 하위 작업

## 진행 상황 요약
**완료율**: 50% (1/2)
**상태**: 🔄 in_progress
```

### 4. Deprecation 전략 (즉시 적용)

**하드 deprecation**: 기존 7개 툴을 **즉시 제거**하고 `tasks` 툴만 사용하도록 변경

- 기존 7개 툴 (`tasks_init`, `tasks_add`, `tasks_update`, `tasks_complete`, `tasks_remove`, `tasks_list`, `tasks_status`) 완전 제거
- 단일 `tasks` 툴만 사용 가능
- 마이그레이션 가이드 문서 제공 (기존 사용법 → 새 사용법)

## Implementation Plan

### Phase 1: Core Implementation
1. `tasks` 툴 구현 (기존 `tasks_batch` 기반)
2. 세션 격리 로직 구현
3. 통합 출력 포맷터 구현

### Phase 2: Deprecation
1. 기존 7개 툴에 deprecation warning 추가
2. 마이그레이션 가이드 작성
3. 하위호환성 테스트

### Phase 3: Cleanup (향후 v4.0.0)
1. 기존 7개 툴 제거
2. 코드 정리

## Acceptance Criteria

- [ ] `tasks` 툴이 모든 작업 유형(init, add, update, complete, remove)을 지원
- [ ] `tasks` 툴 호출 시 현재 세션의 작업만 표시
- [ ] 배치 작업(여러 operation)을 한 번에 처리 가능
- [ ] 부분적 실패 허용 (일부 작업 실패 시 성공한 작업은 유지)
- [ ] 모든 작업 후 현재 세션의 상태 자동 표시
- [ ] 기존 7개 툴에 deprecation warning 표시
- [ ] 마이그레이션 가이드 문서 제공

## Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| Breaking Change | High | Deprecation period 2-3개월 제공 |
| 사용자 학습 곡선 | Medium | 상세한 마이그레이션 가이드 및 예제 제공 |
| 하위호환성 깨짐 | High | Phase별 점진적 도입 |
| 세션 격리 오류 | High | 철저한 테스트 및 검증 |

## References

- 기존 `tasks_batch` 구현: `src/commands/batch.ts`
- 세션 관리: `src/lib/storage.ts`
- 출력 포맷터: `src/lib/formatter.ts`
