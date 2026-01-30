# 작업 목록: 서브에이전트 Task 관리 방식 개선 (플러그인 방식)

**제안 ID**: `update-subagent-task-management`  
**생성일**: 2026-01-30  
**상태**: Planned  
**버전**: 2.0 (플러그인 방식)

---

## 작업 개요

이 작업 목록은 서브에이전트들의 Task 관리 방식을 `todowrite`/`todoread`에서 **tasks 플러그인** 기반으로 변경하는 모든 단계를 정의합니다.

플러그인이 파일 IO, 파싱, 형식 관리 등의 복잡한 작업을 추상화하고, 에이전트는 간단한 명령어 인터페이스만 사용합니다.

---

## 작업 목록 (Task List)

### Phase 1: tasks 플러그인 개발

#### Scenario: 플러그인 기본 구조 설정
**작업 ID**: P1  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] `~/.config/opencode/custom-plugins/tasks/` 디렉토리 생성
- [ ] TypeScript 프로젝트 초기화 (`npm init -y`)
- [ ] TypeScript 설정 (`tsconfig.json`) 작성
- [ ] OpenCode 플러그인 매니페스트 (`manifest.json`) 작성
- [ ] package.json 의존성 설정
- [ ] src/ 디렉토리 구조 생성
- [ ] README.md 초안 작성

**수용 기준**:
- TypeScript 플러그인 디렉토리 구조가 생성됨
- 기본 설정 파일(package.json, tsconfig.json, manifest.json)이 작성됨

---

#### Scenario: 파일 저장소 모듈 개발
**작업 ID**: P2  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] `src/lib/storage.ts` 개발
- [ ] TypeScript 파일 IO 인터페이스 정의
- [ ] 파일 생성 기능 구현 (fs.promises)
- [ ] 파일 읽기 기능 구현
- [ ] 파일 수정 기능 구현
- [ ] 파일 삭제 기능 구현
- [ ] 디렉토리 구조 관리 (`~/.config/opencode/tasks/{agent}/`)

**수용 기준**:
- 파일 CRUD 작업이 정상적으로 동작함
- 디렉토리 구조가 올바르게 생성됨

---

#### Scenario: 마크다운 파서 개발
**작업 ID**: P3  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] `src/lib/parser.ts` 개발
- [ ] TypeScript 인터페이스 정의 (TaskDetail, TaskList, Metadata)
- [ ] 마크다운 파일 파싱 기능 구현
- [ ] 체크박스 상태 추출 기능 구현
- [ ] 작업 목록 추출 기능 구현
- [ ] **작업 세부사항(details) 추출 기능 구현**
  - 들여쓰기된 짧은 문장 리스트 파싱
  - 세부사항 추가/수정/삭제 기능
- [ ] 메타데이터 추출 기능 구현
- [ ] 마크다운 생성 기능 구현 (세부사항 포함)

**수용 기준**:
- 마크다운 파일이 올바르게 파싱됨
- 체크박스 상태가 정확히 추출됨

---

#### Scenario: 핵심 명령어 구현 - init
**작업 ID**: P4  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] `src/commands/init.ts` 구현
- [ ] OpenCode 플러그인 커맨드 인터페이스 정의
- [ ] `--agent`, `--title`, `--file` 옵션 처리
- [ ] 마크다운 템플릿 생성
- [ ] storage 모듈 연동

**수용 기준**:
- `tasks init` 명령어가 정상적으로 동작함
- 올바른 마크다운 파일이 생성됨

---

#### Scenario: 핵심 명령어 구현 - list
**작업 ID**: P5  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] `src/commands/list.ts` 구현
- [ ] OpenCode 플러그인 커맨드 인터페이스 정의
- [ ] `--agent`, `--format` 옵션 처리
- [ ] 마크다운 출력 포맷 구현
- [ ] JSON 출력 포맷 구현 (선택적)
- [ ] Table 출력 포맷 구현 (선택적)
- [ ] formatter 모듈 연동

**수용 기준**:
- `tasks list` 명령어가 정상적으로 동작함
- 다양한 출력 포맷이 지원됨

---

#### Scenario: 핵심 명령어 구현 - update
**작업 ID**: P6  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] `src/commands/update.ts` 구현
- [ ] OpenCode 플러그인 커맨드 인터페이스 정의
- [ ] `--agent`, `--id`, `--status` 옵션 처리
- [ ] 작업 상태 변경 로직 구현
- [ ] storage 및 parser 모듈 연동

**수용 기준**:
- `tasks update` 명령어가 정상적으로 동작함
- 작업 상태가 올바르게 변경됨

---

#### Scenario: 핵심 명령어 구현 - complete
**작업 ID**: P7  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] `src/commands/complete.ts` 구현
- [ ] OpenCode 플러그인 커맨드 인터페이스 정의
- [ ] `--agent`, `--id` 옵션 처리
- [ ] 체크박스 체크 로직 구현
- [ ] storage 및 parser 모듈 연동

**수용 기준**:
- `tasks complete` 명령어가 정상적으로 동작함
- 체크박스가 올바르게 체크됨

---

#### Scenario: 핵심 명령어 구현 - status
**작업 ID**: P8  
**우선순위**: Medium  
**예상 소요시간**: 30분

**요구사항**:
- [ ] `src/commands/status.ts` 구현
- [ ] OpenCode 플러그인 커맨드 인터페이스 정의
- [ ] `--agent` 옵션 처리
- [ ] 진행 상황 요약 정보 추출
- [ ] 완료율 계산
- [ ] 요약 출력 포맷 구현
- [ ] formatter 모듈 연동

**수용 기준**:
- `tasks status` 명령어가 정상적으로 동작함
- 진행 상황 요약이 정확히 출력됨

---

#### Scenario: 핵심 명령어 구현 - remove
**작업 ID**: P8.5  
**우선순위**: Medium  
**예상 소요시간**: 30분

**요구사항**:
- [ ] `src/commands/remove.ts` 구현
- [ ] OpenCode 플러그인 커맨드 인터페이스 정의
- [ ] `--agent`, `--id`, `--force` 옵션 처리
- [ ] 작업 항목 제거 로직 구현
- [ ] 확인 메시지 처리 (force가 아닌 경우)
- [ ] storage 및 parser 모듈 연동

**수용 기준**:
- `tasks remove` 명령어가 정상적으로 동작함
- 작업 항목이 올바르게 제거됨
- 확인 메시지가 적절히 표시됨

---

#### Scenario: 메인 진입점 및 타입 정의 개발
**작업 ID**: P9  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] `src/index.ts` 개발 (OpenCode 플러그인 진입점)
- [ ] `src/types/index.ts` 타입 정의
- [ ] OpenCode 플러그인 API 인터페이스 구현
- [ ] 명령어 등록 및 라우팅
- [ ] 에러 처리 및 로깅
- [ ] 빌드 설정 (dist/ 출력)

**수용 기준**:
- OpenCode 플러그인 API가 올바르게 구현됨
- 모든 명령어가 플러그인을 통해 실행됨
- TypeScript 컴파일이 정상적으로 동작함

---

#### Scenario: TypeScript 빌드 및 플러그인 문서화
**작업 ID**: P10  
**우선순위**: Medium  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] TypeScript 컴파일 설정 검증
- [ ] `npm run build` 빌드 테스트
- [ ] dist/ 출력 확인
- [ ] README.md 완성
- [ ] 설치 방법 문서화 (npm install, OpenCode 플러그인 등록)
- [ ] 사용 방법 문서화
- [ ] 명령어 레퍼런스 작성
- [ ] TypeScript 타입 문서화
- [ ] 예시 포함

**수용 기준**:
- TypeScript 빌드가 정상적으로 동작함
- 플러그인 사용법이 명확히 문서화됨
- OpenCode 플러그인 등록 방법이 설명됨

---

### Phase 2: 템플릿 업데이트

#### Scenario: agent-template.md 플러그인 가이드라인 추가
**작업 ID**: T1  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] agent-template.md 파일 읽기
- [ ] Frontmatter에서 todowrite/todoread 제거 또는 false로 설정
- [ ] `bash: true` 활성화 (플러그인 호출용)
- [ ] 워크플로우 섹션 업데이트 (플러그인 기반 Task 관리로 변경)
- [ ] 가이드라인 섹션에 플러그인 사용법 추가
- [ ] 플러그인 명령어 예시 포함

**수용 기준**:
- agent-template.md에 플러그인 기반 Task 관리 가이드라인이 명확히 문서화됨
- 새로운 서브에이전트 생성 시 자동으로 해당 가이드라인이 포함됨

---

### Phase 3: 기존 서브에이전트 업데이트

#### Scenario: senior-sw-engineer.md 업데이트
**작업 ID**: T2  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- senior-sw-engineer.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: py-code-reviewer.md 업데이트
**작업 ID**: T3  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- py-code-reviewer.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: qa.md 업데이트
**작업 ID**: T4  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- qa.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: cpp-review.md 업데이트
**작업 ID**: T5  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- cpp-review.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: build.md 업데이트
**작업 ID**: T6  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- build.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: plan.md 업데이트
**작업 ID**: T7  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- plan.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: general.md 업데이트
**작업 ID**: T8  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- general.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

#### Scenario: research-analyst.md 업데이트
**작업 ID**: T9  
**우선순위**: High  
**예상 소요시간**: 30분

**요구사항**:
- [ ] Frontmatter에서 todowrite/todoread 제거
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션의 모든 todowrite/todoread 참조를 플러그인 명령어로 변경
- [ ] 가이드라인 섹션의 Todo Management 설명 업데이트
- [ ] 플러그인 사용 예시 추가

**수용 기준**:
- research-analyst.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 모든 워크플로우 단계가 플러그인 명령어로 문서화됨

---

### Phase 4: agent-creator 업데이트

#### Scenario: agent-creator.md 업데이트
**작업 ID**: T10  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] agent-creator.md 파일 읽기
- [ ] Frontmatter에서 todowrite/todoread 제거 (필요시)
- [ ] `bash: true` 활성화
- [ ] 워크플로우 섹션 업데이트 (플러그인 기반 Task 관리로 변경)
- [ ] 가이드라인 섹션에 플러그인 사용법 추가
- [ ] agent-template.md를 참조하여 새로운 서브에이전트 생성 시 동일한 지침 적용 확인

**수용 기준**:
- agent-creator.md가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 새로운 서브에이전트 생성 시 자동으로 플러그인 기반 Task 관리 지침이 포함됨

---

### Phase 5: 검증 및 테스트

#### Scenario: 플러그인 단위 테스트
**작업 ID**: V1  
**우선순위**: High  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] 각 명령어 동작 검증
- [ ] 마크다운 파싱 정확성 검증
- [ ] 파일 생성/수정/삭제 동작 검증
- [ ] 에러 처리 검증

**수용 기준**:
- 모든 명령어가 정상적으로 동작함
- 에러가 적절히 처리됨

---

#### Scenario: 업데이트된 파일 검증
**작업 ID**: V2  
**우선순위**: Medium  
**예상 소요시간**: 1시간

**요구사항**:
- [ ] 모든 서브에이전트 파일에서 todowrite/todoread 참조 제거 확인
- [ ] 플러그인 기반 Task 관리 가이드라인이 모든 파일에 포함되어 있는지 확인
- [ ] `bash: true`가 활성화되어 있는지 확인
- [ ] 플러그인 명령어 예시가 제공되어 있는지 확인

**수용 기준**:
- 모든 서브에이전트가 플러그인 기반 Task 관리를 사용하도록 업데이트됨
- 일관된 가이드라인과 예시가 제공됨

---

#### Scenario: 통합 테스트
**작업 ID**: V3  
**우선순위**: Medium  
**예상 소요시간**: 30분

**요구사항**:
- [ ] 샘플 Task 파일 생성 테스트 (`tasks init`)
- [ ] 작업 목록 조회 테스트 (`tasks list`)
- [ ] 상태 업데이트 테스트 (`tasks update`, `tasks complete`)
- [ ] 진행 상황 조회 테스트 (`tasks status`)

**수용 기준**:
- 플러그인이 정상적으로 동작함
- 워크플로우가 플러그인 기반 방식으로 원활히 진행됨

---

## 의존성 그래프 (Dependency Graph)

```
Phase 1: 플러그인 개발
├── P1 (기본 구조)
│   └── P2 (파일 저장소)
│       └── P3 (마크다운 파서)
│           ├── P4 (init 명령어)
│           ├── P5 (list 명령어)
│           ├── P6 (update 명령어)
│           ├── P7 (complete 명령어)
│           ├── P8 (status 명령어)
│           └── P8.5 (remove 명령어)
│               └── P9 (메인 실행 파일)
│                   └── P10 (문서화)

Phase 2: 템플릿 업데이트
└── P9
    └── T1 (agent-template.md)

Phase 3: 서브에이전트 업데이트
└── T1
    ├── T2 (senior-sw-engineer.md)
    ├── T3 (py-code-reviewer.md)
    ├── T4 (qa.md)
    ├── T5 (cpp-review.md)
    ├── T6 (build.md)
    ├── T7 (plan.md)
    ├── T8 (general.md)
    └── T9 (research-analyst.md)

Phase 4: agent-creator 업데이트
└── T1, T2~T9
    └── T10 (agent-creator.md)

Phase 5: 검증 및 테스트
└── T10
    ├── V1 (플러그인 단위 테스트)
    ├── V2 (파일 검증)
    └── V3 (통합 테스트)
```

---

## 일정 (Schedule)

| 단계 | 작업 | 예상 소요시간 | 의존성 |
|------|------|--------------|--------|
| 1 | P1~P10: TypeScript 플러그인 개발 | 10시간 | 없음 |
| 2 | T1: agent-template.md 업데이트 | 1시간 | P9 |
| 3 | T2~T9: 서브에이전트 업데이트 | 4시간 (병렬) | T1 |
| 4 | T10: agent-creator.md 업데이트 | 1시간 | T1, T2~T9 |
| 5 | V1~V3: 검증 및 테스트 | 3시간 | T10 |

**총 예상 소요시간**: 약 19시간 (병렬 작업 고려)

---

## 리스크 및 완화 방안 (Risks & Mitigations)

| 리스크 | 영향도 | 완화 방안 |
|--------|--------|----------|
| 플러그인 개발 지연 | 높음 | MVP 먼저 개발, 핵심 기능 우선 구현 |
| 플러그인 버그 | 중간 | 충분한 테스트, 폴백 메커니즘 제공 |
| 에이전트 학습 곡선 | 중간 | 명확한 문서화, 예시 제공 |
| 누락된 업데이트 | 중간 | 체크리스트 기반 검증 프로세스 |

---

## 완료 기준 (Definition of Done)

- [ ] tasks 플러그인 개발 완료 (핵심 명령어 구현)
- [ ] 모든 9개 서브에이전트 파일 업데이트 완료
- [ ] agent-template.md 업데이트 완료
- [ ] agent-creator.md 업데이트 완료
- [ ] 모든 파일에서 todowrite/todoread 참조 제거
- [ ] 플러그인 기반 Task 관리 가이드라인이 모든 파일에 포함
- [ ] 플러그인 문서화 완료
- [ ] 검증 및 테스트 완료
