# Tasks Plugin Tools Guide

이 문서는 OpenCode Tasks Plugin에서 제공하는 도구들의 사용 방법을 설명합니다.

## 개요

Tasks Plugin은 에이전트가 작업을 체계적으로 관리하고 추적할 수 있도록 하는 7개의 도구를 제공합니다.

---

## 도구 목록

### 1. tasks_init

작업 목록을 초기화합니다. tasks.md 파일에서 작업 목록을 생성합니다.

**사용법:**
```
tasks_init(agent="[에이전트-이름]", title="[작업-목록-제목]", file="[tasks.md-경로]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름 (예: senior-sw-engineer, py-code-reviewer)
- `title` (필수): 작업 목록 제목
- `file` (필수): tasks.md 파일 경로

**예시:**
```
tasks_init(agent="senior-sw-engineer", title="API-구현", file="./changes/update-api/tasks.md")
```

---

### 2. tasks_list

에이전트의 모든 작업 목록을 조회합니다.

**사용법:**
```
tasks_list(agent="[에이전트-이름]", format="[형식]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름
- `format` (선택): 출력 형식 (markdown, json, table). 기본값: markdown

**예시:**
```
tasks_list(agent="senior-sw-engineer", format="markdown")
```

---

### 3. tasks_update

특정 작업의 상태를 업데이트합니다.

**사용법:**
```
tasks_update(agent="[에이전트-이름]", id="[작업-ID]", status="[상태]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름
- `id` (필수): 작업 ID (예: task-1, task-2.1)
- `status` (필수): 새 상태 (pending, in_progress, completed)

**예시:**
```
tasks_update(agent="senior-sw-engineer", id="task-1", status="in_progress")
tasks_update(agent="senior-sw-engineer", id="task-2", status="completed")
```

---

### 4. tasks_complete

작업을 완료 상태로 표시합니다 (tasks_update의 단축키).

**사용법:**
```
tasks_complete(agent="[에이전트-이름]", id="[작업-ID]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름
- `id` (필수): 작업 ID

**예시:**
```
tasks_complete(agent="senior-sw-engineer", id="task-1")
```

---

### 5. tasks_add

기존 작업 목록에 새 작업을 추가합니다.

**사용법:**
```
tasks_add(agent="[에이전트-이름]", title="[작업-제목]", details=["[세부사항1]", "[세부사항2]"], parent="[부모-작업-ID]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름
- `title` (필수): 작업 제목
- `details` (선택): 세부 작업 목록 (문자열 배열)
- `parent` (선택): 부모 작업 ID (하위 작업으로 추가시)

**예시:**
```
tasks_add(agent="senior-sw-engineer", title="에러 처리 구현", details=["예외 클래스 정의", "로깅 추가", "테스트 작성"])
tasks_add(agent="senior-sw-engineer", title="하위 작업", parent="task-1")
```

---

### 6. tasks_remove

작업을 목록에서 제거합니다.

**사용법:**
```
tasks_remove(agent="[에이전트-이름]", id="[작업-ID]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름
- `id` (필수): 제거할 작업 ID

**예시:**
```
tasks_remove(agent="senior-sw-engineer", id="task-3")
```

---

### 7. tasks_status

에이전트의 전체 작업 진행 상황 요약을 확인합니다.

**사용법:**
```
tasks_status(agent="[에이전트-이름]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름

**예시:**
```
tasks_status(agent="senior-sw-engineer")
```

**출력 예시:**
```
📊 Task Status for senior-sw-engineer

📋 API-구현
Status: in_progress (67% complete)
Completed: 2 / 3 tasks
Current Phase: 구현 단계
```

---

## 워크플로우 예시

### 일반적인 작업 흐름

1. **작업 초기화**
   ```
   tasks_init(agent="senior-sw-engineer", title="기능-구현", file="./tasks.md")
   ```

2. **작업 상태 업데이트 (진행 중)**
   ```
   tasks_update(agent="senior-sw-engineer", id="task-1", status="in_progress")
   ```

3. **작업 완료**
   ```
   tasks_complete(agent="senior-sw-engineer", id="task-1")
   ```

4. **진행 상황 확인**
   ```
   tasks_status(agent="senior-sw-engineer")
   ```

---

## 에이전트 설정

에이전트가 Tasks 도구를 사용하려면 frontmatter에 `tasks_*` 와일드카드를 추가하세요.

```yaml
---
tools:
  tasks_*: true
---
```

**참고**: `tasks_*` 와일드카드는 다음 7개의 도구를 모두 포함합니다:
- `tasks_init`
- `tasks_list`
- `tasks_update`
- `tasks_complete`
- `tasks_add`
- `tasks_remove`
- `tasks_status`

---

## 파일 저장 위치

작업 목록은 다음 위치에 저장됩니다:

```
~/.config/opencode/tasks/{agent-name}/{task-title}.md
```

예시:
```
~/.config/opencode/tasks/senior-sw-engineer/API-구현.md
```

---

## 모범 사례

1. **작업 시작 시 초기화**: 작업을 시작할 때 항상 `tasks_init`로 작업 목록을 초기화하세요.
2. **상태 실시간 업데이트**: 작업 상태가 변경될 때마다 `tasks_update`로 업데이트하세요.
3. **완료 표시**: 작업이 완료되면 `tasks_complete`로 명확히 표시하세요.
4. **진행 상황 모니터링**: 주기적으로 `tasks_status`로 전체 진행 상황을 확인하세요.
5. **세부 작업 분리**: 큰 작업은 `tasks_add`로 세부 작업으로 분리하세요.

---

## 문제 해결

### 작업 목록을 찾을 수 없음
- `tasks_init`을 먼저 실행했는지 확인하세요.
- 에이전트 이름이 올바른지 확인하세요.

### 작업 ID 형식
- 작업 ID는 `task-1`, `task-2.1` 등의 형식을 사용합니다.
- 하위 작업은 `task-1.1`, `task-1.2` 등으로 표현됩니다.

### 권한 문제
- `~/.config/opencode/tasks/` 디렉토리에 쓰기 권한이 있는지 확인하세요.
