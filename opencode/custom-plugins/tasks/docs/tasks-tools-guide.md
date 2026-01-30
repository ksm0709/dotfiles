# Tasks Plugin Tools Guide

이 문서는 OpenCode Tasks Plugin에서 제공하는 도구들의 사용 방법을 설명합니다.

## 개요

Tasks Plugin은 에이전트가 작업을 체계적으로 관리하고 추적할 수 있도록 하는 7개의 도구를 제공합니다.

**주요 특징:**
- 세션 ID는 OpenCode 컨텍스트에서 자동으로 추출됩니다 (에이전트가 입력할 필요 없음)
- 모든 도구는 반환값을 통해 결과를 전달합니다 (console.log 사용 없음)
- TUI 깨짐 없이 안전하게 사용 가능합니다

---

## 도구 목록

### 1. tasks_init

작업 목록을 초기화합니다.

**사용법:**
```typescript
tasks_init(agent="[에이전트-이름]", title="[작업-목록-제목]")
```

**매개변수:**
- `agent` (필수): 에이전트 이름 (예: senior-sw-engineer, py-code-reviewer)
- `title` (필수): 작업 목록 제목

**반환값:**
- `title`: 작업 목록 제목
- `agent`: 에이전트 이름
- `fileName`: 생성된 파일명
- `taskIds`: 사용 가능한 작업 ID 목록
- `totalTasks`: 전체 작업 수

**예시:**
```typescript
const result = tasks_init(agent="senior-sw-engineer", title="API-구현")
// 응답:
// ✅ Task list "API-구현" initialized successfully for agent "senior-sw-engineer"
// 📁 File: senior-sw-engineer-api-구현.md
// 📊 Total tasks: 0
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

**반환값:**
- `success`: 성공 여부
- `taskLists`: TaskList 객체 배열
- `formattedOutput`: 포맷팅된 출력 문자열
- `message`: 상태 메시지

**예시:**
```typescript
const result = tasks_list(format="markdown")
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

**반환값:**
- `success`: 성공 여부
- `taskId`: 작업 ID
- `status`: 업데이트된 상태
- `message`: 상태 메시지

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

**반환값:**
- `success`: 성공 여부
- `taskId`: 작업 ID
- `message`: 상태 메시지

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

**반환값:**
- `success`: 성공 여부
- `title`: 작업 제목
- `parent`: 부모 작업 ID
- `details`: 세부사항 배열
- `message`: 상태 메시지

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

**반환값:**
- `success`: 성공 여부
- `taskId`: 작업 ID
- `taskTitle`: 작업 제목
- `message`: 상태 메시지

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

**반환값:**
- `success`: 성공 여부
- `summaries`: TaskStatusSummary 객체 배열
- `formattedOutput`: 포맷팅된 출력 문자열
- `message`: 상태 메시지

**예시:**
```typescript
const result = tasks_status()
```

**출력 예시:**
```
📊 Task Status Summary

📋 API-구현
Status: in_progress (67% complete)
Completed: 2 / 3 tasks
Current Phase: 구현 단계
```

---

## 워크플로우 예시

### 일반적인 작업 흐름

1. **작업 초기화**
   ```typescript
   const initResult = tasks_init(agent="senior-sw-engineer", title="기능-구현")
   // 작업 ID 확인: initResult.taskIds
   ```

2. **작업 추가**
   ```typescript
   tasks_add(title="요구사항 분석")
   tasks_add(title="설계")
   tasks_add(title="구현")
   ```

3. **작업 상태 업데이트 (진행 중)**
   ```typescript
   tasks_update(id="1", status="in_progress")
   ```

4. **작업 완료**
   ```typescript
   tasks_complete(id="1")
   ```

5. **진행 상황 확인**
   ```typescript
   const statusResult = tasks_status()
   ```

6. **작업 목록 조회**
   ```typescript
   const listResult = tasks_list(format="markdown")
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

1. **작업 시작 시 초기화**: 작업을 시작할 때 항상 `tasks_init`로 작업 목록을 초기화하세요.
2. **작업 ID 활용**: `tasks_init` 반환값의 `taskIds`를 확인하여 적절한 parent ID를 사용하세요.
3. **상태 실시간 업데이트**: 작업 상태가 변경될 때마다 `tasks_update`로 업데이트하세요.
4. **완료 표시**: 작업이 완료되면 `tasks_complete`로 명확히 표시하세요.
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

### v2.0.0 (2026-01-30)
- **BREAKING CHANGE**: `sessionId` 파라미터 제거 (OpenCode 컨텍스트에서 자동 추출)
- **BREAKING CHANGE**: `tasks_init`에서 `file` 파라미터 제거
- **개선**: 모든 도구가 반환값을 통해 결과 전달 (console.log 제거)
- **개선**: TUI 깨짐 문제 해결
- **개선**: XDG Base Directory 준수 (`~/.local/share/opencode/tasks/`)
