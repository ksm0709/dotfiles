# tasks - Task Management Plugin for OpenCode

OpenCode 에이전트를 위한 작업 관리 플러그인입니다. 서브에이전트가 체계적으로 작업을 관리할 수 있도록 지원합니다.

## 📋 기능

- ✅ 작업 목록 초기화 및 관리
- ✅ 작업 상태 추적 (pending → in_progress → completed)
- ✅ 하위 작업(subtask) 지원
- ✅ 다양한 출력 포맷 (markdown, json, table)
- ✅ 진행 상황 요약 및 완료율 계산
- ✅ 세션 기반 자동 저장 (OpenCode 세션 ID 활용)

## 🚀 설치

### 1. 의존성 설치

```bash
cd ~/.config/opencode/custom-plugins/tasks
npm install
```

### 2. TypeScript 컴파일

```bash
npm run build
```

### 3. OpenCode 플러그인 등록

`~/.config/opencode/config.json`에 플러그인을 등록합니다:

```json
{
  "plugins": [
    {
      "name": "tasks",
      "path": "~/.config/opencode/custom-plugins/tasks"
    }
  ]
}
```

## 📖 사용법

### 작업 목록 초기화

```typescript
tasks_init(
  agent="senior-sw-engineer",
  title="implement-auth"
)
// 응답: 작업 목록 정보와 사용 가능한 작업 ID 목록
```

### 작업 목록 조회

```typescript
// 마크다운 형식
tasks_list(format="markdown")

// JSON 형식
tasks_list(format="json")

// 테이블 형식
tasks_list(format="table")
```

### 작업 상태 업데이트

```typescript
tasks_update(
  id="2.1",
  status="in_progress"
)
```

### 작업 완료 처리

```typescript
tasks_complete(id="2.1")
```

### 작업 제거

```typescript
tasks_remove(id="2.1")
```

### 진행 상황 요약

```typescript
tasks_status()
```

### 새 작업 추가

```typescript
// 최상위 작업 추가
tasks_add(title="에러 처리 구현")

// 하위 작업 추가
tasks_add(
  title="예외 클래스 정의",
  parent="2"
)
```

## 📁 파일 저장 구조

```
~/.local/share/opencode/tasks/
├── {session-id-1}/
│   ├── senior-sw-engineer-implement-auth.md
│   └── senior-sw-engineer-fix-bug-123.md
├── {session-id-2}/
│   ├── py-code-reviewer-review-pr-456.md
│   └── qa-verify-login.md
└── ...
```

## 📝 마크다운 파일 형식

```markdown
# Task List: implement-user-auth

**에이전트**: senior-sw-engineer  
**생성일**: 2026-01-30 14:30:00  
**세션 ID**: abc-123-def

---

## 작업 목록

- [x] 1. 요구사항 분석
- [ ] 2. 기능 구현
  - [x] 2.1 핵심 로직 개발
  - [ ] 2.2 테스트 코드 작성

---

## 진행 상황 요약

**현재 단계**: 2. 기능 구현  
**상태**: in_progress  
**완료율**: 40% (2/5)
```

## 🔧 개발

### 빌드

```bash
npm run build
```

### 개발 모드 (watch)

```bash
npm run dev
```

### 클린

```bash
npm run clean
```

### 테스트

```bash
npm test
```

## 📄 라이선스

MIT
