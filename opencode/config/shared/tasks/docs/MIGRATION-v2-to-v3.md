# Migration Guide: v2 → v3

## 개요

Tasks Plugin v3.0에서는 **7개의 개별 툴이 1개의 통합 `tasks` 툴로 통합**되었습니다. 이 가이드는 기존 v2 코드를 v3로 마이그레이션하는 방법을 설명합니다.

## 주요 변경사항

### 1. 툴 통합

| v2 (개별 툴) | v3 (통합 툴) |
|-------------|-------------|
| `tasks_init()` | `tasks({ operations: [{ type: 'init', ... }] })` |
| `tasks_add()` | `tasks({ operations: [{ type: 'add', ... }] })` |
| `tasks_update()` | `tasks({ operations: [{ type: 'update', ... }] })` |
| `tasks_complete()` | `tasks({ operations: [{ type: 'complete', ... }] })` |
| `tasks_remove()` | `tasks({ operations: [{ type: 'remove', ... }] })` |
| `tasks_list()` | 자동 표시 (모든 작업 후 현재 상태) |
| `tasks_status()` | 자동 표시 (모든 작업 후 현재 상태) |

### 2. 세션 격리

- **v2**: 모든 세션의 작업이 `tasks_list`, `tasks_status`에 표시됨
- **v3**: 현재 세션의 작업만 표시됨 (다른 세션 완전 격리)

### 3. 출력 형식

- **v2**: 툴별로 다른 출력, `tasks_list`와 `tasks_status` 별도 호출 필요
- **v3**: 모든 작업 후 자동으로 현재 세션의 작업 목록과 상태 표시

## 마이그레이션 예제

### 예제 1: 작업 목록 초기화

**v2 (이전):**
```typescript
tasks_init(agent="senior-sw-engineer", title="API-구현")
```

**v3 (새로운):**
```typescript
tasks({
  operations: [
    { type: 'init', agent: 'senior-sw-engineer', title: 'API-구현' }
  ]
})
```

### 예제 2: 작업 추가

**v2 (이전):**
```typescript
tasks_add(title="요구사항 분석")
tasks_add(title="설계 문서 작성")
tasks_add(title="구현", parent="2")
```

**v3 (새로운):**
```typescript
tasks({
  operations: [
    { type: 'add', title: '요구사항 분석' },
    { type: 'add', title: '설계 문서 작성' },
    { type: 'add', title: '구현' }
  ]
})
```

**참고:** v3.0+에서는 parent/child 개념이 제거되었습니다. 모든 작업은 flat한 구조로 관리됩니다. 필요시 작업 제목에 계층 정보를 포함하세요 (예: "1.1. 구현").

### 예제 3: 상태 업데이트

**v2 (이전):**
```typescript
tasks_update(id="1", status="in_progress")
tasks_complete(id="2")
```

**v3 (새로운):**
```typescript
tasks({
  operations: [
    { type: 'update', id: '1', status: 'in_progress' },
    { type: 'complete', id: '2' }
  ]
})
```

### 예제 4: 전체 워크플로우

**v2 (이전):**
```typescript
// 초기화
tasks_init(agent="dev", title="프로젝트")

// 작업 추가
tasks_add(title="작업 1")
tasks_add(title="작업 2")
tasks_add(title="하위 작업", parent="2")  // v3: parent 파라미터 제거됨

// 상태 업데이트
tasks_update(id="1", status="completed")
tasks_complete(id="2")

// 현황 확인
tasks_list(format="markdown")
tasks_status()
```

**v3 (새로운) - 배치 처리:**
```typescript
tasks({
  operations: [
    { type: 'init', agent: 'dev', title: '프로젝트' },
    { type: 'add', title: '작업 1' },
    { type: 'add', title: '작업 2' },
    { type: 'add', title: '2.1. 하위 작업' },  // flat 구조: ID 형식으로 계층 표현
    { type: 'update', id: '1', status: 'completed' },
    { type: 'complete', id: '2' }
  ]
})
// 자동으로 현재 세션의 작업 현황 표시
```

## Operation 타입 상세

### `init` - 작업 목록 초기화

```typescript
{
  type: 'init',
  agent: string,      // 필수: 에이전트 이름
  title: string       // 필수: 작업 목록 제목
}
```

### `add` - 작업 추가

```typescript
{
  type: 'add',
  title: string       // 필수: 작업 제목
}
```

**참고:** v3.0+에서는 parent 파라미터가 제거되었습니다. 모든 작업은 동일한 레벨(flat)에 추가됩니다. 필요시 작업 제목에 계층 ID를 포함할 수 있습니다 (예: "1.1. 작업명").

### `update` - 상태 업데이트

```typescript
{
  type: 'update',
  id: string,         // 필수: 작업 ID
  status: TaskStatus  // 필수: 'pending' | 'in_progress' | 'completed'
}
```

### `complete` - 작업 완료

```typescript
{
  type: 'complete',
  id: string          // 필수: 작업 ID
}
```

### `remove` - 작업 제거

```typescript
{
  type: 'remove',
  id: string          // 필수: 작업 ID
}
```

## 제한사항 및 주의사항

### 1. 최대 작업 수

- 한 번에 최대 50개의 operation을 처리할 수 있습니다.
- 더 많은 작업이 필요하면 여러 번의 `tasks` 호출로 분리하세요.

### 2. 부분적 실패

- 일부 operation이 실패해도 성공한 operation은 유지됩니다.
- 결과의 `summary` 필드에서 성공/실패 개수를 확인할 수 있습니다.

### 3. 세션 격리

- 현재 세션의 작업만 표시됩니다.
- 다른 세션의 작업을 보려면 해당 세션에서 `tasks`를 호출해야 합니다.

## 문제 해결

### "No task lists found" 오류

**원인:** 작업 목록이 초기화되지 않은 상태에서 `add`, `update` 등을 호출

**해결:**
```typescript
tasks({
  operations: [
    { type: 'init', agent: 'dev', title: '프로젝트' },  // 먼저 초기화
    { type: 'add', title: '작업 1' }                   // 그 다음 추가
  ]
})
```

### "Task not found" 오류

**원인:** 존재하지 않는 작업 ID를 업데이트/완료/제거하려고 시도

**해결:** 작업 ID를 확인하고 올바른 ID를 사용하세요.

### 세션 간 데이터 공유

**v2:** 세션 간 작업 목록이 공유됨
**v3:** 각 세션은 완전히 격리됨

**해결:** 세션 간 데이터 이동이 필요하면 수동으로 파일을 복사하세요.

## 지원 및 피드백

마이그레이션 중 문제가 발생하면 GitHub Issues에 리포트해 주세요.

---

**마이그레이션 완료 후 체크리스트:**
- [ ] 모든 `tasks_*` 호출을 `tasks`로 변경
- [ ] `tasks_list`와 `tasks_status` 호출 제거 (자동 표시됨)
- [ ] 세션 격리로 인한 데이터 접근 문제 확인
- [ ] 배치 작업으로 효율성 개선
