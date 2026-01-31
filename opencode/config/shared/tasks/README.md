# tasks - Unified Task Management Plugin for OpenCode

OpenCode 에이전트를 위한 **통합 작업 관리 플러그인**입니다. 단일 `tasks` 툴로 모든 작업을 관리하며, 세션 격리를 통해 보안과 프라이버시를 강화합니다.

## 🎯 주요 특징 (v3.0)

- ✅ **통합 인터페이스** - 단일 `tasks` 툴로 모든 작업 관리
- ✅ **세션 격리** - 현재 세션의 작업만 표시 (다른 세션 완전 격리)
- ✅ **배치 작업 지원** - 한 번에 여러 작업 처리 (최대 50개)
- ✅ **부분적 실패 허용** - 일부 작업 실패 시 성공한 작업은 유지
- ✅ **자동 현황 표시** - 모든 작업 후 현재 세션 상태 자동 표시
- ✅ **하위 작업(subtask) 지원**
- ✅ **네이티브 UI 렌더링** - OpenCode UI와 완벽 통합

## 🚀 설치

### 자동 설치 (권장)

```bash
cd ~/.config/opencode/custom-plugins/tasks
./install.sh
```

### 수동 설치

```bash
# 플러그인 소스 복사
cp -r src/* ~/.config/opencode/plugins/tasks/

# 문서 복사
mkdir -p ~/.config/opencode/shared/tasks
cp README.md ~/.config/opencode/shared/tasks/
cp -r docs ~/.config/opencode/shared/tasks/
cp -r templates ~/.config/opencode/shared/tasks/
```

## 📁 설치 구조

```
~/.config/opencode/
├── plugins/tasks/              # 실행용 TypeScript 파일
│   ├── index.ts               # 진입점 (통합 tasks 툴)
│   ├── commands/              # 명령어 구현
│   │   ├── unified.ts         # 통합 명령 핸들러
│   │   ├── init.ts            # 초기화 (내부용)
│   │   ├── add-task.ts        # 작업 추가 (내부용)
│   │   ├── update.ts          # 상태 업데이트 (내부용)
│   │   ├── complete.ts        # 완료 처리 (내부용)
│   │   └── remove.ts          # 작업 제거 (내부용)
│   ├── lib/                   # 유틸리티 라이브러리
│   └── types/                 # TypeScript 타입 정의
├── shared/tasks/              # 문서 및 가이드
└── tasks/                     # 작업 데이터 저장소 (세션별 격리)
    └── {session-id}/
        └── {agent}-{title}.md
```

## 🚀 빠른 시작

### 기본 사용법

```typescript
// 작업 목록 초기화
tasks({
  operations: [
    { type: 'init', agent: 'senior-sw-engineer', title: 'API-구현' }
  ]
})

// 작업 추가
tasks({
  operations: [
    { type: 'add', title: '요구사항 분석' },
    { type: 'add', title: '설계 문서 작성' },
    { type: 'add', title: '구현', parent: '2' }
  ]
})

// 상태 업데이트 및 완료
tasks({
  operations: [
    { type: 'update', id: '1', status: 'in_progress' },
    { type: 'complete', id: '2' }
  ]
})
```

### 배치 작업 (권장)

```typescript
tasks({
  operations: [
    { type: 'init', agent: 'dev', title: '프로젝트' },
    { type: 'add', title: '작업 1' },
    { type: 'add', title: '작업 2' },
    { type: 'add', title: '하위 작업', parent: '2' },
    { type: 'update', id: '1', status: 'completed' },
    { type: 'complete', id: '2' }
  ]
})
```

## 📖 사용법

### 1. 작업 목록 초기화 (`init`)

```typescript
tasks({
  operations: [
    { type: 'init', agent: '에이전트명', title: '작업목록제목' }
  ]
})
```

**필수 필드:**
- `agent`: 에이전트 이름 (예: senior-sw-engineer)
- `title`: 작업 목록 제목

### 2. 작업 추가 (`add`)

```typescript
tasks({
  operations: [
    { type: 'add', title: '작업 제목' },
    { type: 'add', title: '하위 작업', parent: '1' }  // parent: 부모 작업 ID
  ]
})
```

**필수 필드:**
- `title`: 작업 제목

**선택 필드:**
- `parent`: 부모 작업 ID (하위 작업으로 추가)

### 3. 상태 업데이트 (`update`)

```typescript
tasks({
  operations: [
    { type: 'update', id: '1', status: 'in_progress' }
  ]
})
```

**상태 값:**
- `pending`: 대기
- `in_progress`: 진행 중
- `completed`: 완료

### 4. 작업 완료 (`complete`)

```typescript
tasks({
  operations: [
    { type: 'complete', id: '1' }
  ]
})
```

`update` with `status: 'completed'`의 단축키입니다.

### 5. 작업 제거 (`remove`)

```typescript
tasks({
  operations: [
    { type: 'remove', id: '1' }
  ]
})
```

## 🔒 세션 격리

**v3.0의 핵심 기능**: 현재 세션의 작업만 표시됩니다.

- 각 세션은 독립적인 작업 공간을 가집니다
- 다른 세션의 작업은 완전히 격리되어 보이지 않습니다
- 세션 ID는 OpenCode 컨텍스트에서 자동 추출됩니다

```
~/.local/share/opencode/tasks/
├── {session-id-1}/          # 세션 1의 작업
│   └── agent-tasks.md
├── {session-id-2}/          # 세션 2의 작업 (완전 격리)
│   └── agent-tasks.md
└── {session-id-3}/          # 세션 3의 작업 (완전 격리)
    └── agent-tasks.md
```

## 📊 출력 형식

모든 `tasks` 호출은 현재 세션의 작업 현황을 자동으로 표시합니다:

```markdown
# 📦 작업 실행 결과

## 📊 요약
- **총 작업**: 6
- **✅ 성공**: 5
- **❌ 실패**: 1

[███████░░░░░░░░░░░░░] 83%

## ✅ 성공한 작업
- ✅ **init**: Task list "프로젝트" initialized successfully
- ✅ **add**: Task added: 작업 1 (ID: 1)
- ✅ **add**: Task added: 작업 2 (ID: 2)
- ✅ **update**: Task 1 status updated to completed
- ✅ **complete**: Task 2 marked as completed

---

# 📋 프로젝트

**에이전트**: dev

## 📊 진행 상황
| 상태 | 개수 | 비율 |
|------|------|------|
| ✅ 완료 | 2 | 100% |
| 🔄 진행중 | 0 | 0% |
| ⏳ 대기 | 0 | 0% |
| **합계** | **2** | **100%** |

### 진행률
[████████████████████] 100%

---

## 📋 작업 목록
- [x] ✅ **1**. 작업 1
- [x] ✅ **2**. 작업 2
```

## 🔄 마이그레이션 가이드 (v2 → v3)

### 이전 버전 (v2) - 개별 툴 사용

```typescript
// ❌ 더 이상 사용되지 않음
tasks_init(agent="dev", title="프로젝트")
tasks_add(title="작업 1")
tasks_add(title="작업 2", parent="1")
tasks_update(id="1", status="completed")
tasks_complete(id="2")
tasks_list(format="markdown")
tasks_status()
```

### 새 버전 (v3) - 통합 툴 사용

```typescript
// ✅ 새로운 방식
tasks({
  operations: [
    { type: 'init', agent: 'dev', title: '프로젝트' },
    { type: 'add', title: '작업 1' },
    { type: 'add', title: '작업 2', parent: '1' },
    { type: 'update', id: '1', status: 'completed' },
    { type: 'complete', id: '2' }
  ]
})
```

**주요 변경사항:**
1. 7개 개별 툴 → 1개 통합 `tasks` 툴
2. 모든 작업 후 자동으로 현재 세션 상태 표시
3. 세션 격리로 다른 세션의 작업이 보이지 않음
4. 배치 작업을 통한 효율적인 작업 관리

## 🛠️ 개발

### 빌드

```bash
npm run build
```

### 테스트

```bash
npm test
```

### 테스트 (커버리지)

```bash
npm run test:coverage
```

## 📄 라이선스

MIT

## 🤝 기여

버그 리포트와 기능 제안은 GitHub Issues를 통해 해주세요.

---

**참고**: 이 플러그인은 OpenCode의 공식 플러그인 구조를 따릅니다.
