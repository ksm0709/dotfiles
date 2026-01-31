<!-- TASKS_TOOLS_GUIDE_START -->
<!-- 이 섹션은 Tasks Plugin에 의해 자동으로 관리됩니다. 수동으로 수정하지 마세요. -->

### 📋 Task Management Tools

Tasks Plugin은 에이전트가 작업을 체계적으로 관리할 수 있도록 다음 도구들을 제공합니다:

#### 사용 가능한 도구 (tasks_*)

Tasks Plugin은 `tasks_*` 와일드카드로 모든 작업 관리 도구를 제공합니다.

**주요 도구:**
- **`tasks_batch(operations)`** (⭐ 권장) - 여러 작업을 한 번에 처리
  - 반환값: `{ title, output, metadata }`
  - metadata: { tasks, taskList, summary, operation, results, batchSummary }
- **`tasks_init(agent, title)`**: 작업 목록 초기화
  - 반환값: `{ title, output, metadata }`
  - metadata: { agent, fileName, taskIds, totalTasks }
- **`tasks_list(format)`**: 작업 목록 조회 (format: markdown/json/table)
  - 반환값: `{ title, output, metadata }`
  - metadata: { success, taskLists, formattedOutput, message }
- **`tasks_update(id, status)`**: 작업 상태 업데이트 (status: pending/in_progress/completed)
  - 반환값: `{ title, output, metadata }`
  - metadata: { success, taskId, status, message, tasks, taskList, summary }
- **`tasks_complete(id)`**: 작업 완료 처리
  - 반환값: `{ title, output, metadata }`
  - metadata: { success, taskId, message, tasks, taskList, summary }
- **`tasks_add(title, parent)`**: 새 작업 추가 (parent는 선택적)
  - 반환값: `{ title, output, metadata }`
  - metadata: { success, title, parent, details, message, tasks, taskList, summary }
- **`tasks_remove(id)`**: 작업 제거
  - 반환값: `{ title, output, metadata }`
  - metadata: { success, taskId, taskTitle, message, tasks, taskList, summary }
- **`tasks_status()`**: 전체 진행 상황 확인
  - 반환값: `{ title, output, metadata }`
  - metadata: { success, summaries, formattedOutput, message }

**사용 예시:**
```typescript
// 작업 목록 초기화
const initResult = tasks_init(agent="senior-sw-engineer", title="API-구현")
// 응답 예시:
// title: "Task list initialized"
// output: "✅ Task list \"API-구현\" initialized..."
// metadata: { agent, fileName, taskIds, totalTasks }

// ⭐ 배치 작업으로 여러 작업 한 번에 처리 (권장)
tasks_batch({
  operations: [
    { type: 'add', title: '요구사항 분석' },
    { type: 'add', title: '설계' },
    { type: 'add', title: '구현' },
    { type: 'update', id: '1', status: 'in_progress' }
  ]
})

// 진행 상황 확인
const statusResult = tasks_status()
```

**주요 특징:**
- ⭐ **배치 작업 권장**: 여러 작업은 `tasks_batch`로 한 번에 처리하세요
- 세션 ID는 OpenCode 컨텍스트에서 자동으로 추출됩니다
- 모든 도구는 ToolResponse 형태로 반환됩니다: `{ title, output, metadata }`
- OpenCode가 metadata를 활용하여 네이티브 UI로 렌더링합니다
- 작업 파일은 `~/.local/share/opencode/tasks/{session-id}/`에 저장됩니다

**설치 구조:**
- 플러그인 코드: `~/.config/opencode/plugins/tasks/`
- 문서 및 가이드: `~/.config/opencode/shared/tasks/`

#### 에이전트 설정

에이전트가 Tasks 도구를 사용하려면 frontmatter에 다음을 추가하세요:

**방법 1: `tasks_*` 와일드카드 사용 (권장)**
모든 Tasks 도구를 한 번에 활성화하려면 `tasks_*` 와일드카드를 사용하세요:

```yaml
---
tools:
  tasks_*: true
---
```

**방법 2: 개별 도구 활성화**
특정 도구만 필요한 경우 개별적으로 지정할 수 있습니다:

```yaml
---
tools:
  tasks_batch: true
  tasks_init: true
  tasks_list: true
  tasks_update: true
  tasks_complete: true
  tasks_add: true
  tasks_remove: true
  tasks_status: true
---
```

#### 자세한 사용법

자세한 사용법은 다음 문서를 참조하세요:
`~/.config/opencode/shared/tasks/docs/tasks-tools-guide.md`

<!-- TASKS_TOOLS_GUIDE_END -->
