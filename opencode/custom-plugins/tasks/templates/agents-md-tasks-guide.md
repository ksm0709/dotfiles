<!-- TASKS_TOOLS_GUIDE_START -->
<!-- 이 섹션은 Tasks Plugin에 의해 자동으로 관리됩니다. 수동으로 수정하지 마세요. -->

### 📋 Task Management Tools

Tasks Plugin은 에이전트가 작업을 체계적으로 관리할 수 있도록 다음 도구들을 제공합니다:

#### 사용 가능한 도구 (tasks_*)

Tasks Plugin은 `tasks_*` 와일드카드로 모든 작업 관리 도구를 제공합니다.

**주요 도구:**
- **`tasks_init(agent, title, file)`**: 작업 목록 초기화
- **`tasks_list(agent, format)`**: 작업 목록 조회
- **`tasks_update(agent, id, status)`**: 작업 상태 업데이트
- **`tasks_complete(agent, id)`**: 작업 완료 처리
- **`tasks_add(agent, title, details, parent)`**: 새 작업 추가
- **`tasks_remove(agent, id)`**: 작업 제거
- **`tasks_status(agent)`**: 전체 진행 상황 확인

**사용 예시:**
```
tasks_init(agent="senior-sw-engineer", title="API-구현", file="./tasks.md")
tasks_update(agent="senior-sw-engineer", id="task-1", status="in_progress")
tasks_complete(agent="senior-sw-engineer", id="task-1")
```

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
`~/.config/opencode/custom-plugins/tasks/docs/tasks-tools-guide.md`

<!-- TASKS_TOOLS_GUIDE_END -->
