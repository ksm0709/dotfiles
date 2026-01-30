# 제안서: 서브에이전트 Task 관리 방식 개선 (플러그인 방식)

## 개요 (Overview)

**제안 ID**: `update-subagent-task-management`  
**제안일**: 2026-01-30  
**제안자**: PM (Project Manager)  
**상태**: Draft  
**버전**: 2.0 (플러그인 방식으로 전환)

---

## 변경 목적 (Purpose)

현재 서브에이전트(subagent)들은 `todowrite`와 `todoread` 도구를 사용하여 작업 목록을 관리하고 있으나, **서브에이전트는 실제로 이 도구들을 사용할 수 없는 제약**이 존재합니다.

초기에는 파일 시스템 기반의 직접 관리 방식을 제안했으나, **에이전트의 인지적 부하를 고려하여 플러그인 방식으로 전환**합니다. 플러그인이 파일 IO, 파싱, 형식 관리 등의 복잡한 작업을 추상화하고, 에이전트는 간단한 명령어 인터페이스만 사용하면 됩니다.

---

## 현재 문제점 (Current Problems)

### 1. 도구 사용 불가
- 서브에이전트는 `todowrite`, `todoread` 도구를 호출할 수 없음
- 이로 인해 작업 계획 수립 및 상태 추적이 불가능함

### 2. 작업 관리의 비일관성
- PM(Primary) 에이전트는 todowrite/todoread 사용 가능
- 서브에이전트는 동일한 방식으로 작업 관리 불가
- 작업 위임 시 컨텍스트 전달 및 추적이 어려움

### 3. 에이전트 인지적 부하
- 파일 직접 관리 방식은 에이전트가 파일 IO, 마크다운 파싱, 형식 관리 등을 직접 처리해야 함
- 이는 에이전트의 인지적 부하를 증가시켜 핵심 작업에 집중하기 어렵게 함

---

## 제안하는 해결책 (Proposed Solution)

### 핵심 아이디어
**`tasks` 플러그인 개발 및 도입**

`~/.config/opencode/custom-plugins/tasks/`에 플러그인을 개발하여, 모든 서브에이전트가 일관된 인터페이스로 Task를 관리하도록 합니다.

### 플러그인 위치
```
~/.config/opencode/custom-plugins/tasks/
├── package.json          # NPM 패키지 설정
├── tsconfig.json         # TypeScript 설정
├── manifest.json         # OpenCode 플러그인 매니페스트
├── src/
│   ├── index.ts          # 메인 진입점
│   ├── commands/         # 명령어 구현
│   │   ├── init.ts
│   │   ├── list.ts
│   │   ├── update.ts
│   │   ├── complete.ts
│   │   ├── remove.ts
│   │   └── status.ts
│   ├── lib/
│   │   ├── taskManager.ts   # 핵심 로직
│   │   ├── formatter.ts     # 출력 포맷터
│   │   └── storage.ts       # 파일 저장소
│   └── types/
│       └── index.ts      # 타입 정의
├── dist/                 # 컴파일된 출력 (빌드 결과)
└── README.md             # 문서
```

### 명령어 인터페이스

#### 핵심 명령어
```bash
# 작업 목록 생성
tasks init --agent {name} --title {title} --file {tasks-md-file}

# 작업 목록 조회
tasks list --agent {name} [--format {json|markdown|table}]

# 작업 상태 업데이트
tasks update --agent {name} --id {task-id} --status {status}

# 작업 완료
tasks complete --agent {name} --id {task-id}

# 작업 제거
tasks remove --agent {name} --id {task-id} [--force]

# 진행 상황 요약
tasks status --agent {name}
```

### 파일 형식 및 데이터 모델

각 작업(task)은 제목뿐만 아니라 **짧은 문장의 리스트로 된 세부사항**도 함께 저장합니다:

```markdown
## 작업 목록

- [x] 1. 요구사항 분석
  - 사용자 인증 요구사항 정의
  - 보안 표준 및 규제 준수 확인
- [ ] 2. 기능 구현
  - [x] 2.1 핵심 로직 개발
    - JWT 토큰 생성 및 검증 구현
    - 비밀번호 해싱 로직 구현
  - [ ] 2.2 테스트 코드 작성
    - 단위 테스트 커버리지 80% 이상
    - 통합 테스트 시나리오 작성
```

**TypeScript 타입 정의**:
```typescript
interface TaskDetail {
  id: string;
  title: string;
  status: 'pending' | 'in_progress' | 'completed';
  details: string[];  // 작업 세부사항 리스트
  subtasks?: TaskDetail[];
}
```

### 파일 저장 구조
```
~/.config/opencode/tasks/
├── senior-sw-engineer/
│   ├── implement-user-auth.md
│   └── fix-bug-123.md
├── py-code-reviewer/
│   ├── review-pr-456.md
│   └── review-pr-789.md
└── qa/
    ├── verify-login.md
    └── test-payment.md
```

---

## 변경 범위 (Scope)

### ## ADDED

#### 1. tasks 플러그인 개발
- **위치**: `~/.config/opencode/custom-plugins/tasks/`
- **내용**:
  - 핵심 명령어 구현 (init, list, update, complete, status)
  - 마크다운 파일 생성 및 파싱
  - 파일 저장소 관리
  - 출력 포맷터 (markdown, json, table)

### ## MODIFIED

#### 2. 서브에이전트 도구 설정
- **대상**: 9개 서브에이전트 파일
  - `senior-sw-engineer.md`
  - `py-code-reviewer.md`
  - `qa.md`
  - `cpp-review.md`
  - `build.md`
  - `plan.md`
  - `general.md`
  - `research-analyst.md`
  - `agent-creator.md`
- **변경 내용**:
  - `todowrite: true` → `todowrite: false` (또는 제거)
  - `todoread: true` → `todoread: false` (또는 제거)
  - `bash: true` 활성화 (플러그인 호출용)
  - 플러그인 기반 Task 관리 가이드라인 추가

#### 3. 워크플로우 문서 업데이트
- **대상**: 모든 서브에이전트의 Workflow 섹션
- **변경 내용**:
  - `todowrite` 호출 → `tasks init` 명령어
  - `todoread` 호출 → `tasks list` 명령어
  - 상태 업데이트 → `tasks update` 또는 `tasks complete` 명령어

#### 4. 가이드라인 업데이트
- **대상**: 모든 서브에이전트의 Guidelines 섹션
- **변경 내용**:
  - Todo Management 가이드라인을 플러그인 기반으로 변경
  - 플러그인 설치 및 사용법 문서화

#### 5. agent-template.md 업데이트
- **대상**: `~/.config/opencode/templates/agent-template.md`
- **변경 내용**:
  - 새로운 서브에이전트 생성 시 플러그인 기반 Task 관리 가이드라인 포함

#### 6. agent-creator.md 업데이트
- **대상**: `~/.config/opencode/agent/agent-creator.md`
- **변경 내용**:
  - 서브에이전트 생성 시 플러그인 기반 Task 관리 지침 자동 포함

---

## 기대 효과 (Expected Benefits)

1. **인지적 부하 감소**: 에이전트는 복잡한 파일 IO를 신경 쓸 필요 없이 간단한 명령어만 사용
2. **작업 관리의 일관성**: 모든 에이전트가 동일한 인터페이스로 작업 추적
3. **상태 가시성**: 외부에서 `~/.config/opencode/tasks/` 파일을 통해 진행 상황 실시간 확인 가능
4. **유지보수성**: 파일 형식 변경 시 플러그인만 수정하면 됨
5. **확장성**: 새로운 기능(자동 정리, 통계 등)을 플러그인에 쉽게 추가 가능
6. **디버깅 용이성**: 작업 이력이 파일로 남아 문제 추적이 쉬움

---

## 수용 기준 (Acceptance Criteria)

- [ ] tasks 플러그인 개발 완료 (핵심 명령어 구현)
- [ ] 모든 서브에이전트가 `todowrite`/`todoread` 대신 플러그인 사용
- [ ] 플러그인 명령어 인터페이스가 문서화됨
- [ ] 파일 저장 구조 `~/.config/opencode/tasks/{agent}/{title}.md` 준수
- [ ] agent-template.md에 플러그인 기반 Task 관리 가이드라인 포함
- [ ] agent-creator.md가 새로운 서브에이전트에 동일한 지침 적용

---

## 관련 문서 (References)

- [design.md](./design.md): 상세 설계 문서
- [tasks.md](./tasks.md): 구현 작업 목록
