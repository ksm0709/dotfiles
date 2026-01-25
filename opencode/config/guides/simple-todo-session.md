# Simple Todo Manager - 세션별 독립된 상태 공간

## 개요
**세션별 독립된 상태 공간**을 제공하는 Simple Todo Manager입니다. 각 서브 에이전트는 자신만의 세션을 가지며, 다른 세션과 완전히 격리된 환경에서 Todo를 관리할 수 있습니다.

## 특징
- ✅ **세션별 격리**: 각 세션은 독립된 디렉토리와 상태 공간 유지
- ✅ **자동 세션 ID**: 지정하지 않으면 자동으로 고유한 세션 ID 생성
- ✅ **독립적 사용**: PM이나 다른 agent의 도움 불필요
- ✅ **세션 관리**: 세션 정보 조회, 정리, 목록 기능 제공
- ✅ **상태 추적**: pending, in_progress, completed 상태 관리

## 사용법

### 1. 기본 사용법 (자동 세션 ID)
```python
# Simple Todo Manager 생성 - 자동으로 세션 ID 생성
from tools.simple_todo import SimpleTodoManager

todos = SimpleTodoManager("your-agent-name")  # 예: "senior-sw-engineer"

# Todo 추가
task_id = todos.add_todo("작업 내용", "high")

# 상태 업데이트
todos.update_status(task_id, "in_progress")
todos.update_status(task_id, "completed")

# 목록 조회
todos.list_todos()

# 세션 정보 조회
session_info = todos.get_session_info()
print(f"세션 ID: {session_info['session_id']}")
print(f"저장 위치: {session_info['todo_dir']}")
```

### 2. 수동 세션 ID 지정
```python
# 특정 세션 ID로 생성
todos = SimpleTodoManager("senior-sw-engineer", "project-alpha")

# 동일한 agent지만 다른 세션 (완전히 독립된 공간)
todos2 = SimpleTodoManager("senior-sw-engineer", "project-beta")
```

### 3. 실제 사용 예시

#### Senior SW Engineer - 여러 프로젝트 세션
```python
# 프로젝트 Alpha 세션
sse_alpha = SimpleTodoManager("senior-sw-engineer", "project-alpha")
task1 = sse_alpha.add_todo("Alpha API 구현", "high")
sse_alpha.update_status(task1, "in_progress")

# 프로젝트 Beta 세션 (독립된 공간)
sse_beta = SimpleTodoManager("senior-sw-engineer", "project-beta")
task2 = sse_beta.add_todo("Beta 데이터베이스 설계", "medium")

# 각 세션은 완전히 독립됨
print(f"Alpha 세션: {len(sse_alpha.load_todos())}개 Todo")
print(f"Beta 세션: {len(sse_beta.load_todos())}개 Todo")
```

#### Py Code Reviewer - 자동 세션
```python
# 자동 세션 ID 생성
pyr = SimpleTodoManager("py-code-reviewer")  # 자동으로 "8df7e6bf" 같은 ID 생성

review_task = pyr.add_todo("코드 품질 검토", "high")
pyr.update_status(review_task, "in_progress")

# 세션 정보 확인
info = pyr.get_session_info()
print(f"세션 ID: {info['session_id']}")
print(f"진행 중: {info['in_progress']}개")
```

### 4. 세션 관리 기능

#### 모든 세션 목록 조회
```python
# 모든 agent의 모든 세션 목록
all_sessions = SimpleTodoManager.list_all_sessions()
for session in all_sessions:
    print(f"{session['agent_name']} ({session['session_id']}) - {session['total_todos']}개 Todo")

# 특정 agent의 세션만 조회
sse_sessions = SimpleTodoManager.list_all_sessions("senior-sw-engineer")
for session in sse_sessions:
    print(f"SSE 세션: {session['session_id']}")
```

#### 세션 정리
```python
# 세션 완료 후 정리
todos = SimpleTodoManager("senior-sw-engineer", "temp-session")
# ... 작업 수행 ...
todos.cleanup_session()  # 세션 디렉토리 완전 삭제
```

## 저장 구조

### 디렉토리 구조
```
/tmp/agent_sessions/
├── senior-sw-engineer_project-alpha/
│   └── todos.json
├── senior-sw-engineer_project-beta/
│   └── todos.json
├── py-code-reviewer_8df7e6bf/
│   └── todos.json
└── qa_b7efb5a6/
    └── todos.json
```

### Todo 파일 형식
```json
[
  {
    "id": "sen-001",
    "content": "API 엔드포인트 구현",
    "priority": "high",
    "status": "in_progress",
    "created_at": "2026-01-24T10:51:29.749694",
    "updated_at": "2026-01-24T10:51:29.758589",
    "session_id": "project-alpha"
  }
]
```

## 상태 값
- `pending`: 대기 중 (⏳)
- `in_progress`: 진행 중 (🔄)
- `completed`: 완료됨 (✅)

## 우선순위
- `high`: 높음 (🔴)
- `medium`: 보통 (🟡)
- `low`: 낮음 (🟢)

## 장점
1. **완전한 격리**: 각 세션은 다른 세션과 완전히 독립
2. **유연성**: 동일한 agent가 여러 세션 동시 운영 가능
3. **자동화**: 세션 ID 자동 생성 및 관리
4. **추적성**: 세션별 상태 정보 및 메타데이터 제공
5. **정리 기능**: 세션 완료 후 깔끔한 정리 지원

## 사용 시나리오

### 1. 여러 프로젝트 동시 진행
```python
# 프로젝트 A
project_a = SimpleTodoManager("senior-sw-engineer", "project-a")

# 프로젝트 B  
project_b = SimpleTodoManager("senior-sw-engineer", "project-b")

# 각 프로젝트는 완전히 독립된 Todo 목록 유지
```

### 2. 임시 작업 세션
```python
# 임시 테스트 세션
temp_session = SimpleTodoManager("qa", "temp-test-001")
# ... 테스트 작업 수행 ...
temp_session.cleanup_session()  # 작업 완료 후 정리
```

### 3. 장기 세션 관리
```python
# 모든 세션 상태 확인
all_sessions = SimpleTodoManager.list_all_sessions()
for session in all_sessions:
    if session['total_todos'] == 0:
        print(f"빈 세션: {session['session_id']} - 정리 고려")
```

## 🎯 핵심 철학
**"각 서브 에이전트는 세션별 독립된 상태 공간에서 완벽하게 격리된 작업 수행"**