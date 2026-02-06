# Tasks: Tasks 플러그인 버그 수정 및 세션 완료 체크 기능 구현

## 작업 개요

이 문서는 Tasks 플러그인의 `in_progress` 상태 버그 수정과 세션 종료 시 미완료 task 확인 기능 구현을 위한 작업 목록입니다.

---

## Phase 1: 버그 수정 - in_progress 상태 전환

### Task 1: Parser 수정 - parseTaskList 메서드 업데이트
**파일**: `src/lib/parser.ts`
**우선순위**: 높음
**예상 소요시간**: 30분

#### 작업 내용
- [x] taskMatch 정규식 수정: `[ x]` → `[ x~]`
- [x] 상태 파싱 로직 추가: `~` → `in_progress`
- [x] 하위 호환성 유지: `[ ]` → `pending` (기본값)

#### 시나리오
```typescript
// 테스트 케이스
expect(parseCheckbox('[x]')).toBe('completed');
expect(parseCheckbox('[~]')).toBe('in_progress');
expect(parseCheckbox('[ ]')).toBe('pending');
```

---

### Task 2: Parser 수정 - formatTask 메서드 업데이트
**파일**: `src/lib/parser.ts`
**우선순위**: 높음
**예상 소요시간**: 20분

#### 작업 내용
- [x] 상태별 체크박스 생성 로직 수정
- [x] `completed` → `[x]`
- [x] `in_progress` → `[~]`
- [x] `pending` → `[ ]`

#### 시나리오
```typescript
// 테스트 케이스
expect(formatCheckbox('completed')).toBe('[x]');
expect(formatCheckbox('in_progress')).toBe('[~]');
expect(formatCheckbox('pending')).toBe('[ ]');
```

---

### Task 3: updateCommand 검증 및 수정
**파일**: `src/commands/update.ts`
**우선순위**: 중간
**예상 소요시간**: 20분

#### 작업 내용
- [x] updateCommand가 `in_progress` 상태를 올바르게 처리하는지 검증
- [x] 필요시 수정 (현재로서는 Parser 수정만으로 해결될 것으로 예상)

---

### Task 4: 버그 수정 테스트 작성
**파일**: `tests/lib/parser.test.ts`
**우선순위**: 높음
**예상 소요시간**: 40분

#### 작업 내용
- [x] `parseTaskList` in_progress 파싱 테스트
- [x] `generateTaskList` in_progress 생성 테스트
- [x] 통합: update → save → parse 테스트

#### 시나리오
- **Scenario**: in_progress 상태 업데이트 성공
  - Given: 세션에 "1. 테스트 작업" task가 pending 상태로 존재
  - When: 사용자가 `update` 명령으로 id "1"의 상태를 `in_progress`로 변경 요청
  - Then: task 상태가 `in_progress`로 변경되고 마크다운 파일에 반영됨

- **Scenario**: in_progress 상태 파싱 확인
  - Given: 마크다운 파일에 `[~]` 표시의 task가 저장됨
  - When: Parser가 해당 파일을 파싱
  - Then: task의 status가 `in_progress`로 인식됨

---

## Phase 2: 세션 종료 체크 기능

### Task 5: CompletionChecker 모듈 구현
**파일**: `src/lib/completion-checker.ts` (신규)
**우선순위**: 높음
**예상 소요시간**: 40분

#### 작업 내용
- [x] `CompletionChecker` 클래스 생성
- [x] `checkIncompleteTasks(sessionId)` 메서드 구현
- [x] 미완료 task (pending, in_progress) 식별 로직
- [x] 요약 정보 생성 (pending, inProgress, completed 카운트)

#### 시나리오
```typescript
// 테스트 케이스
const result = await checker.checkIncompleteTasks('session-123');
expect(result.hasIncomplete).toBe(true);
expect(result.summary.inProgress).toBe(1);
expect(result.summary.pending).toBe(2);
```

---

### Task 6: PromptGenerator 모듈 구현
**파일**: `src/lib/prompt-generator.ts` (신규)
**우선순위**: 높음
**예상 소요시간**: 30분

#### 작업 내용
- [x] `PromptGenerator` 클래스 생성
- [x] `generateIncompleteTaskPrompt(tasks, summary)` 메서드 구현
- [x] 한국어 프롬프트 템플릿 작성
- [x] task 목록 포맷팅 (이모지, 상태 표시)

#### 프롬프트 예시
```
⚠️ **작업 완료 알림**

현재 세션에 완료되지 않은 작업이 있습니다.
- 🔄 진행 중: 1개
- ⏳ 대기 중: 2개
- ✅ 완료됨: 1개

**남은 작업 목록:**

🔄 **2**. 작업 B (진행 중)
⏳ **3**. 작업 C (대기 중)
⏳ **4**. 작업 D (대기 중)

📌 **이 작업들을 완료해주세요.**
완료 후 `tasks` 도구를 사용하여 상태를 업데이트하세요.
```

---

### Task 7: 세션 종료 이벤트 핸들러 구현 (사용자 입력 주입 방식)
**파일**: `src/index.ts`
**우선순위**: 높음
**예상 소요시간**: 50분

#### 작업 내용
- [x] OpenCode `session.idle` 이벤트 구독 (세션 유휴 상태 감지)
- [x] 세션 유휴 상태 시 `CompletionChecker` 실행
- [x] 미완료 task 있을 경우 **사용자가 입력한 것처럼 프롬프트를 세션으로 직접 주입**
- [x] OpenCode client API 사용하여 메시지 주입 (예: `ctx.client.session.prompt()`)
- [x] 주입된 메시지가 에이전트의 다음 동작을 트리거하도록 구현

#### ⚠️ 중요: 프롬프트 전송 방식
**단순 반환이 아닌 사용자 입력 주입**: 생성된 프롬프트는 마치 사용자가 직접 입력한 것처럼 세션으로 전송되어야 하며, 이를 통해 에이전트가 자동으로 다음 동작을 수행하게 됩니다.

```typescript
// 구현 예시
await ctx.client.session.prompt(sessionId, {
  role: 'user',
  content: prompt  // 사용자가 입력한 것처럼 주입
});
```

#### 시나리오
- **Scenario**: 미완료 task가 있는 세션 종료
  - Given: 세션에 완료되지 않은 task (pending: 2개, in_progress: 1개)가 존재
  - When: `session.idle` 이벤트 발생
  - Then: 
    1. 미완료 task 목록 수집
    2. 프롬프트를 **사용자 입력처럼 세션에 주입**
    3. 에이전트가 주입된 메시지를 받아 작업 완료 유도 동작 수행

- **Scenario**: 모든 task가 완료된 세션 종료
  - Given: 세션의 모든 task가 completed 상태
  - When: `session.idle` 이벤트 발생
  - Then: 프롬프트 주입 없이 정상 종료

---

### Task 8: 타입 정의 추가
**파일**: `src/types/index.ts`
**우선순위**: 중간
**예상 소요시간**: 15분

#### 작업 내용
- [x] `CompletionCheckResult` 인터페이스 추가
- [x] 관련 타입 정의 추가

---

### Task 9: 세션 종료 체크 테스트 작성
**파일**: `tests/lib/completion-checker.test.ts` (신규)
**우선순위**: 높음
**예상 소요시간**: 40분

#### 작업 내용
- [x] 미완료 task 식별 테스트
- [x] 빈 세션 처리 테스트
- [x] 다중 task 목록 처리 테스트
- [x] 요약 정보 정확성 테스트

---

### Task 10: 프롬프트 생성 테스트 작성
**파일**: `tests/lib/prompt-generator.test.ts` (신규)
**우선순위**: 중간
**예상 소요시간**: 30분

#### 작업 내용
- [x] 프롬프트 형식 검증 테스트
- [x] 한국어 메시지 확인 테스트
- [x] task 목록 정렬 및 표시 테스트

---

## Phase 3: 통합 및 문서화

### Task 11: 통합 테스트
**파일**: `tests/integration/session-completion.test.ts` (신규)
**우선순위**: 높음
**예상 소요시간**: 40분

#### 작업 내용
- [x] end-to-end 세션 종료 시나리오 테스트
- [x] 버그 수정 + 세션 체크 통합 테스트

---

### Task 12: 문서 업데이트
**파일**: `README.md`, `docs/tasks-tools-guide.md`
**우선순위**: 중간
**예상 소요시간**: 30분

#### 작업 내용
- [x] `in_progress` 상태 사용법 문서화
- [x] 세션 종료 체크 기능 설명 추가
- [x] 마크다운 상태 표기법 설명 (`[~]` = in_progress)

---

### Task 13: 린트 및 타입 체크
**명령**: `npm run lint`, `npm run type-check`
**우선순위**: 높음
**예상 소요시간**: 10분

#### 작업 내용
- [x] ESLint 검사 통과
- [x] TypeScript 타입 체크 통과
- [x] Prettier 포맷팅 적용

---

## Phase 4: 검증 및 배포

### Task 14: 코드 리뷰
**검토자**: py-code-reviewer.md
**우선순위**: 높음
**예상 소요시간**: 30분

#### 작업 내용
- [x] 코드 품질 검토
- [x] 보안 취약점 검사
- [x] 성능 병목 분석

---

### Task 15: QA 검증
**검증자**: qa.md
**우선순위**: 높음
**예상 소요시간**: 30분

#### 작업 내용
- [x] OpenSpec 시나리오 기반 기능 검증
- [x] 모든 테스트 통과 확인

---

### Task 16: 최종 아카이빙
**명령**: `openspec-archive`
**우선순위**: 중간
**예상 소요시간**: 10분

#### 작업 내용
- [x] 변경사항 아카이빙
- [x] 사용자에게 완료 보고

---

## 작업 요약

| Phase | 작업 수 | 예상 총 소요시간 |
|-------|---------|------------------|
| Phase 1: 버그 수정 | 4개 | ~2시간 |
| Phase 2: 세션 체크 기능 | 6개 | ~3시간 |
| Phase 3: 통합 및 문서화 | 3개 | ~1.5시간 |
| Phase 4: 검증 및 배포 | 3개 | ~1시간 |
| **총계** | **16개** | **~7.5시간** |

---

## 의존성 그래프

```
Task 1 (parseTaskList 수정)
    ↓
Task 2 (formatTask 수정)
    ↓
Task 4 (버그 수정 테스트)
    ↓
Task 3 (updateCommand 검증) ←→ Task 11 (통합 테스트)

Task 5 (CompletionChecker) ←→ Task 8 (타입 정의)
    ↓
Task 6 (PromptGenerator)
    ↓
Task 7 (이벤트 핸들러)
    ↓
Task 9 (체크 테스트) / Task 10 (프롬프트 테스트)
    ↓
Task 11 (통합 테스트)
    ↓
Task 12 (문서 업데이트) → Task 13 (린트/타입 체크)
    ↓
Task 14 (코드 리뷰)
    ↓
Task 15 (QA 검증)
    ↓
Task 16 (아카이빙)
```
