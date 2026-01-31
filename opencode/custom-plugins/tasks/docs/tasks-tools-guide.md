# Tasks Plugin Tools Guide

이 문서는 OpenCode Tasks Plugin에서 제공하는 도구들의 사용 방법을 설명합니다.

## 개요

Tasks Plugin은 에이전트가 작업을 체계적으로 관리하고 추적할 수 있도록 하는 8개의 도구를 제공합니다.

**주요 특징:**
- ⭐ **배치 작업 권장**: 여러 작업은 `tasks_batch`로 한 번에 처리하세요
- 세션 ID는 OpenCode 컨텍스트에서 자동으로 추출됩니다 (에이전트가 입력할 필요 없음)
- 모든 도구는 ToolResponse 형태로 반환됩니다: `{ title, output, metadata }`
- OpenCode가 metadata를 활용하여 네이티브 UI로 렌더링합니다
- TUI 깨짐 없이 안전하게 사용 가능합니다

---

## 도구 목록

### 0. tasks_batch (권장)

여러 작업을 한 번에 처리합니다. 효율적이고 강력한 배치 작업 도구입니다.

**사용법:**
```typescript
tasks_batch({
  operations: [
    { type: 'add', title: '작업 제목', parent: '부모ID' },
    { type: 'update', id: '작업ID', status: 'pending|in_progress|completed' },
    { type: 'complete', id: '작업ID' },
    { type: 'remove', id: '작업ID' }
  ]
})
```

**매개변수:**
- `operations` (필수): 작업 배열
  - `type`: 'add' | 'update' | 'complete' | 'remove'
  - `title` (add 시 필수): 작업 제목
  - `parent` (add 시 선택): 부모 작업 ID
  - `id` (update/complete/remove 시 필수): 작업 ID
  - `status` (update 시 필수): 새 상태

**반환값 (ToolResponse):**
- `title`: 작업 결과 요약
- `output`: 마크다운 형식의 상세 출력
- `metadata`: OpenCode UI 렌더링용 구조화된 데이터
  - `tasks`: 전체 작업 배열
  - `taskList`: 작업 목록 객체
  - `summary`: 통계 정보
  - `operation`: 'batch'
  - `results`: 개별 작업 결과 배열
  - `batchSummary`: { total, succeeded, failed }

**예시:**
```typescript
const result = tasks_batch({
  operations: [
    { type: 'add', title: '요구사항 분석' },
    { type: 'add', title: '설계' },
    { type: 'update', id: '1', status: 'in_progress' }
  ]
})
// 응답:
// title: "Batch: 3 succeeded, 0 failed"
// metadata.tasks: [작업1, 작업2, ...]
// metadata.summary: { completed: 0, inProgress: 1, pending: 1 }
```

**장점:**
- ✅ 한 번의 호출로 여러 작업 처리
- ✅ 네트워크/통신 오버헤드 감소
- ✅ 트랜잭션 단위로 작업 관리
- ✅ 부분적 실패 허용 (일부 실패해도 성공한 작업 유지)

---

### 1. tasks_init

작업 목록을 초기화합니다.

**사용법:**
```typescript
tasks_init(agent="[에이전트-이름]", title="[작업-목록-제목]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름 (예: senior-sw-engineer, py-code-reviewer)
- `title` (필수): 작업 목록 제목

**반환값 (ToolResponse):**
- `title`: 작업 목록 제목
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `agent`: 에이전트 이름
  - `fileName`: 생성된 파일명
  - `taskIds`: 사용 가능한 작업 ID 목록
  - `totalTasks`: 전체 작업 수

**예시:**
```typescript
const result = tasks_init(agent="senior-sw-engineer", title="API-구현")
// 응답:
// title: "Task list initialized"
// output: "✅ Task list \"API-구현\" initialized..."
// metadata: { agent, fileName, taskIds, totalTasks }
```

---

### 2. tasks_list

현재 세션의 모든 작업 목록을 조회합니다.

**사용법:**
```typescript
tasks_list(format="[형식]")
```

**매개변수:**
- `format` (선택): 출력 형식 (markdown, json, table). 기본값: markdown

**반환값 (ToolResponse):**
- `title`: 결과 요약
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `success`: 성공 여부
  - `taskLists`: TaskList 객체 배열
  - `formattedOutput`: 포맷팅된 출력 문자열
  - `message`: 상태 메시지

**예시:**
```typescript
const result = tasks_list(format="markdown")
// 응답:
// title: "Task Lists"
// output: 마크다운 형식의 작업 목록
// metadata: { taskLists, summary }
```

---

### 3. tasks_update

특정 작업의 상태를 업데이트합니다.

**사용법:**
```typescript
tasks_update(id="[작업-ID]", status="[상태]")
```

**매개변수:**
- `id` (필수): 작업 ID (예: 1, 2.1)
- `status` (필수): 새 상태 (pending, in_progress, completed)

**반환값 (ToolResponse):**
- `title`: 작업 결과 요약
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `success`: 성공 여부
  - `taskId`: 작업 ID
  - `status`: 업데이트된 상태
  - `message`: 상태 메시지
  - `tasks`: 전체 작업 배열
  - `taskList`: 작업 목록 객체
  - `summary`: 통계 정보

**예시:**
```typescript
tasks_update(id="1", status="in_progress")
tasks_update(id="2", status="completed")
```

---

### 4. tasks_complete

작업을 완료 상태로 표시합니다 (tasks_update의 단축키).

**사용법:**
```typescript
tasks_complete(id="[작업-ID]")
```

**매개변수:**
- `id` (필수): 작업 ID

**반환값 (ToolResponse):**
- `title`: 작업 결과 요약
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `success`: 성공 여부
  - `taskId`: 작업 ID
  - `message`: 상태 메시지
  - `tasks`: 전체 작업 배열
  - `taskList`: 작업 목록 객체
  - `summary`: 통계 정보

**예시:**
```typescript
tasks_complete(id="1")
```

---

### 5. tasks_add

기존 작업 목록에 새 작업을 추가합니다.

**사용법:**
```typescript
tasks_add(title="[작업-제목]", parent="[부모-작업-ID]")
```

**매개변수:**
- `title` (필수): 작업 제목
- `parent` (선택): 부모 작업 ID (하위 작업으로 추가시)

**반환값 (ToolResponse):**
- `title`: 작업 결과 요약
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `success`: 성공 여부
  - `title`: 작업 제목
  - `parent`: 부모 작업 ID
  - `details`: 세부사항 배열
  - `message`: 상태 메시지
  - `tasks`: 전체 작업 배열
  - `taskList`: 작업 목록 객체
  - `summary`: 통계 정보

**예시:**
```typescript
// 최상위 작업 추가
tasks_add(title="에러 처리 구현")

// 하위 작업 추가
tasks_add(title="예외 클래스 정의", parent="1")
```

---

### 6. tasks_remove

작업을 목록에서 제거합니다.

**사용법:**
```typescript
tasks_remove(id="[작업-ID]")
```

**매개변수:**
- `id` (필수): 제거할 작업 ID

**반환값 (ToolResponse):**
- `title`: 작업 결과 요약
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `success`: 성공 여부
  - `taskId`: 작업 ID
  - `taskTitle`: 작업 제목
  - `message`: 상태 메시지
  - `tasks`: 전체 작업 배열
  - `taskList`: 작업 목록 객체
  - `summary`: 통계 정보

**예시:**
```typescript
tasks_remove(id="3")
```

---

### 7. tasks_status

현재 세션의 전체 작업 진행 상황 요약을 확인합니다.

**사용법:**
```typescript
tasks_status()
```

**매개변수:**
- 없음

**반환값 (ToolResponse):**
- `title`: 결과 요약
- `output`: 마크다운 형식의 출력
- `metadata`:
  - `success`: 성공 여부
  - `summaries`: TaskStatusSummary 객체 배열
  - `formattedOutput`: 포맷팅된 출력 문자열
  - `message`: 상태 메시지

**예시:**
```typescript
const result = tasks_status()
// 응답:
// title: "Task Status Summary"
// output: "📊 Task Status Summary\n\n📋 API-구현\nStatus: in_progress (67% complete)..."
// metadata: { summaries }
```

---

## 워크플로우 예시

### 권장 워크플로우 (Batch 사용)

1. **작업 목록 초기화**
   ```typescript
   const initResult = tasks_init(agent="senior-sw-engineer", title="기능-구현")
   ```

2. **배치 작업으로 여러 작업 한 번에 처리** ⭐
   ```typescript
   tasks_batch({
     operations: [
       { type: 'add', title: '요구사항 분석' },
       { type: 'add', title: '설계 문서 작성' },
       { type: 'add', title: '구현' },
       { type: 'add', title: '테스트', parent: '3' }
     ]
   })
   ```

3. **작업 상태 업데이트 (진행 중)**
   ```typescript
   tasks_batch({
     operations: [
       { type: 'update', id: '1', status: 'in_progress' }
     ]
   })
   // 또는 개별 툴 사용:
   // tasks_update(id="1", status="in_progress")
   ```

4. **작업 완료**
   ```typescript
   tasks_batch({
     operations: [
       { type: 'complete', id: '1' },
       { type: 'complete', id: '2' }
     ]
   })
   ```

5. **진행 상황 확인**
   ```typescript
   const statusResult = tasks_status()
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

**참고**: `tasks_*` 와일드카드는 다음 8개의 도구를 모두 포함합니다:
- `tasks_batch` (⭐ 권장)
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
~/.local/share/opencode/tasks/{session-id}/{agent-name}-{task-title}.md
```

예시:
```
~/.local/share/opencode/tasks/abc-123-def/senior-sw-engineer-API-구현.md
```

**참고**: `session-id`는 OpenCode 컨텍스트에서 자동으로 추출됩니다.

---

## 플러그인 설치 구조

Tasks Plugin은 OpenCode 공식 구조에 따라 다음과 같이 설치됩니다:

```
~/.config/opencode/
├── plugins/tasks/              # 실행용 TypeScript 파일
│   ├── index.ts               # 진입점
│   ├── commands/              # 명령어 구현 (init, list, update 등)
│   ├── lib/                   # 유틸리티 라이브러리 (storage, parser 등)
│   └── types/                 # TypeScript 타입 정의
├── shared/tasks/              # 문서 및 가이드 (참조용)
│   ├── README.md              # 기본 설명서
│   ├── docs/
│   │   └── tasks-tools-guide.md   # 상세 가이드 (이 문서)
│   └── templates/
│       └── agents-md-tasks-guide.md   # AGENTS.md 템플릿
└── tasks/                     # 작업 데이터 저장소 (자동 생성)
    └── {session-id}/
        └── {agent}-{title}.md
```

### 설치 방법

**자동 설치:**
```bash
cd ~/.config/opencode/custom-plugins/tasks
./install.sh
```

**수동 설치:**
```bash
# 1. 플러그인 소스 복사
cp -r src/* ~/.config/opencode/plugins/tasks/

# 2. 문서 복사
mkdir -p ~/.config/opencode/shared/tasks
cp README.md ~/.config/opencode/shared/tasks/
cp -r docs ~/.config/opencode/shared/tasks/
cp -r templates ~/.config/opencode/shared/tasks/

# 3. 의존성 추가
# ~/.config/opencode/package.json에 "uuid": "^11.1.0" 추가
```

---

## 모범 사례

1. **배치 작업 우선 사용**: 여러 작업을 처리할 때는 항상 `tasks_batch`를 먼저 고려하세요.
   - 개별 툴(tasks_add, tasks_update 등)은 단일 작업에만 사용하세요.

2. **작업 시작 시 초기화**: 작업을 시작할 때 항상 `tasks_init`로 작업 목록을 초기화하세요.

3. **상태 실시간 업데이트**: 작업 상태가 변경될 때마다 업데이트하세요.
   - batch로 한 번에 여러 상태 업데이트 가능

4. **완료 표시**: 작업이 완료되면 명확히 표시하세요.

5. **진행 상황 모니터링**: 주기적으로 `tasks_status`로 전체 진행 상황을 확인하세요.

---

## 문제 해결

### 작업 목록을 찾을 수 없음
- `tasks_init`을 먼저 실행했는지 확인하세요.
- 세션이 변경되지 않았는지 확인하세요 (세션별로 작업 목록이 분리됨).

### 작업 ID 형식
- 작업 ID는 `1`, `2`, `2.1` 등의 형식을 사용합니다.
- 하위 작업은 `1.1`, `1.2` 등으로 표현됩니다.

### 권한 문제
- `~/.local/share/opencode/tasks/` 디렉토리에 쓰기 권한이 있는지 확인하세요.

---

## 업데이트 내역

### v3.0.0 (2026-01-31)
- **신규**: `tasks_batch` 도구 추가 - 한 번에 여러 작업 처리
- **개선**: 모든 도구가 ToolResponse 형태로 반환 (네이티브 UI 지원)
  - `title`, `output`, `metadata` 필드 포함
  - OpenCode가 metadata를 활용하여 UI 렌더링
- **개선**: 작업 후 현황 자동 표시 - 모든 작업 결과에 현재 상태 포함
- **개선**: 마크다운 UI 개선 - 이모지, 테이블, 진행률 바

### v2.0.0 (2026-01-30)
- **BREAKING CHANGE**: `sessionId` 파라미터 제거 (OpenCode 컨텍스트에서 자동 추출)
- **BREAKING CHANGE**: `tasks_init`에서 `file` 파라미터 제거
- **개선**: 모든 도구가 반환값을 통해 결과 전달 (console.log 제거)
- **개선**: TUI 깨짐 문제 해결
- **개선**: XDG Base Directory 준수 (`~/.local/share/opencode/tasks/`)
