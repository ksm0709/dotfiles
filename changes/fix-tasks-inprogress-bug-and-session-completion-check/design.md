# Design: Tasks 플러그인 버그 수정 및 세션 완료 체크 기능

## 1. 개요 (Overview)

이 문서는 Tasks 플러그인의 버그 수정과 새로운 세션 완료 체크 기능의 상세 설계를 기술합니다.

---

## 2. 버그 분석 및 수정 설계

### 2.1 버그 원인 분석

`in_progress` 상태 전환 버그는 다음 중 하나 또는 조합의 원인일 수 있습니다:

1. **Parser의 taskMatch 정규식**: 현재 `parseTaskList` 메서드의 taskMatch는 체크박스만 확인하고 `[x]`인 경우 completed, 아닌 경우 pending으로 간주
   ```typescript
   const taskMatch = line.match(/^- \[([ x])\] ((?:\d+\.)+\d*\.?\s*.+)$/);
   ```
   - 여기서는 `[x]`만 체크하고 있음
   - `in_progress` 상태는 별도의 표시가 필요

2. **마크다운 생성 시 상태 표현**: `formatTask` 메서드는 completed만 체크하고 있음
   ```typescript
   const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
   ```

### 2.2 해결 방안

**방안 1**: in_progress 상태 표현 개선 (권장)

마크다운에서 `in_progress` 상태를 표현하는 방법:

```markdown
- [~] 1. 진행 중인 작업   (in_progress 표시)
- [x] 2. 완료된 작업     (completed)
- [ ] 3. 대기 중인 작업  (pending)
```

수정할 컴포넌트:

1. **parser.ts - parseTaskList**: 체크박스 패턴 확장
   ```typescript
   // AS-IS
   const taskMatch = line.match(/^- \[([ x])\] ((?:\d+\.)+\d*\.?\s*.+)$/);
   
   // TO-BE
   const taskMatch = line.match(/^- \[([ x~])\] ((?:\d+\.)+\d*\.?\s*.+)$/);
   const status = match[1] === 'x' ? 'completed' : 
                 match[1] === '~' ? 'in_progress' : 'pending';
   ```

2. **parser.ts - formatTask**: 상태별 체크박스 생성
   ```typescript
   const checkbox = task.status === 'completed' ? '[x]' : 
                   task.status === 'in_progress' ? '[~]' : '[ ]';
   ```

**하위 호환성**: 기존 파일은 `[ ]`로 저장되어 있으므로, 기본값은 `pending`으로 처리하여 호환성 유지

---

## 3. 세션 종료 체크 기능 설계

### 3.1 아키텍처 개요

```
┌─────────────────────────────────────────────────────────────┐
│                    Session Lifecycle                       │
├─────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────┐     ┌──────────────┐     ┌────────────┐    │
│  │  작업 중  │────▶│ 세션 종료     │────▶│ 체크 로직   │    │
│  │ (Tasks)  │     │ 이벤트 발생   │     │ 실행        │    │
│  └──────────┘     └──────────────┘     └──────┬─────┘    │
│                                                │          │
│                       ┌────────────────────────┘          │
│                       ▼                                   │
│              ┌────────────────┐                          │
│              │ 미완료 Task    │                          │
│              │ 확인           │                          │
│              └───────┬────────┘                          │
│                      │                                    │
│         ┌──────────┴──────────┐                         │
│         ▼                      ▼                         │
│  ┌──────────────┐    ┌──────────────┐                   │
│  │ 미완료 있음  │    │  모두 완료   │                   │
│  │ (Prompt 전송)│    │ (정상 종료)  │                   │
│  └──────────────┘    └──────────────┘                   │
│                                                            │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 세션 종료 이벤트 감지 및 메시지 주입 API 검증

**검증 결과**: OpenCode 플러그인 SDK에서 사용자 입력처럼 세션에 메시지를 주입하는 것이 가능함을 확인했습니다.

#### 검증된 API: `client.session.prompt()`

OpenCode SDK의 `client.session.prompt()` 메서드를 사용하면 플러그인에서 사용자가 입력한 것처럼 메시지를 세션에 전송할 수 있습니다. 이 메시지는 시스템에 의해 "사용자 메시지"로 인식되며 AI 추론을 트리거합니다.

**공식 문서 및 예시 출처**:
- OpenCode Plugins Guide (gist.github.com/johnlindquist)
- OpenCode 공식 문서 (opencode.ai/docs/plugins/)
- GitHub Issue #3378 및 관련 코드 예시

#### 지원되는 세션 이벤트

코드 검색 결과 OpenCode는 다음 세션 이벤트를 지원합니다:
- `session.idle` - 세션이 유휰 상태가 될 때 (작업 완료 후) ⭐ 사용 예정
- `session.created` - 세션 생성 시
- `session.deleted` - 세션 삭제 시
- `session.updated` - 세션 업데이트 시

#### 검증된 구현 패턴

```typescript
// OpenCode 플러그인에서 사용자 입력처럼 메시지 주입 (검증됨)
export const TasksPlugin: Plugin = async ({ client }) => {
  return {
    event: async ({ event }) => {
      // 세션 유휰 상태 감지 (작업 완료 후)
      if (event.type === 'session.idle') {
        const sessionId = event.sessionID;
        
        // 미완료 task 확인
        const storage = new Storage();
        const parser = new Parser();
        const checker = new CompletionChecker(storage, parser);
        const promptGen = new PromptGenerator();
        
        const result = await checker.checkIncompleteTasks(sessionId);
        
        if (result.hasIncomplete) {
          const prompt = promptGen.generateIncompleteTaskPrompt(
            result.incompleteTasks,
            result.summary
          );
          
          // ✅ 검증됨: 사용자 입력처럼 메시지 주입
          // client.session.prompt()는 메시지를 사용자 입력으로 처리하고 AI 추론을 트리거함
          await client.session.prompt({
            path: { id: sessionId },
            body: {
              parts: [{
                type: 'text',
                text: prompt
              }]
            }
          });
          
          // 메시지 주입 후 AI가 "사용자의 요청"으로 인식하여 
          // "작업 완료를 도와드리겠습니다" 등의 응답을 생성함
        }
      }
    },
    // ... tools
  };
};
```

#### 참고: stop hook에서의 메시지 주입 예시 (공식 예시)

```typescript
// OpenCode 공식 문서의 stop hook 예시
stop: async (input) => {
  const sessionId = input.sessionID || input.session_id;
  if (!workComplete) {
    // ✅ 사용자 입력처럼 메시지 주입 (검증된 패턴)
    await client.session.prompt({
      path: { id: sessionId },
      body: {
        parts: [{ type: 'text', text: 'Please complete X before stopping.' }]
      }
    });
  }
}
```

**⚠️ 주의사항**: 
- `client.session.prompt()`는 메시지를 보내고 AI 응답을 기다림 (동기적)
- `session.idle` 이벤트는 세션이 유휰이 된 후 발생하므로 race condition 최소화
- Silent Message Insertion API (#3378)는 AI 추론 없이 메시지만 삽입하는 기능으로, 별도로 요청된 상태 (우리의 use case는 AI 추론을 원하므로 `client.session.prompt()`가 적합)

### 3.3 미완료 Task 확인 로직

```typescript
// src/lib/completion-checker.ts
export class CompletionChecker {
  constructor(private storage: Storage, private parser: Parser) {}

  async checkIncompleteTasks(sessionId: string): Promise<{
    hasIncomplete: boolean;
    incompleteTasks: TaskDetail[];
    summary: { pending: number; inProgress: number; completed: number };
  }> {
    const files = await this.storage.listTaskFiles(sessionId);
    
    if (files.length === 0) {
      return { hasIncomplete: false, incompleteTasks: [], summary: { pending: 0, inProgress: 0, completed: 0 } };
    }

    const incompleteTasks: TaskDetail[] = [];
    let pending = 0, inProgress = 0, completed = 0;

    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await this.storage.readTaskList(sessionId, title);
      if (!content) continue;

      const taskList = this.parser.parseTaskList(content);
      
      for (const task of taskList.tasks) {
        if (task.status === 'pending') {
          pending++;
          incompleteTasks.push(task);
        } else if (task.status === 'in_progress') {
          inProgress++;
          incompleteTasks.push(task);
        } else {
          completed++;
        }
      }
    }

    return {
      hasIncomplete: incompleteTasks.length > 0,
      incompleteTasks,
      summary: { pending, inProgress, completed }
    };
  }
}
```

### 3.4 프롬프트 생성 및 전송 (중요: 사용자 입력처럼 세션으로 직접 전송)

**핵심 동작**: 생성된 프롬프트는 마치 사용자가 직접 입력한 것처럼 에이전트 세션으로 전송되어, 에이전트의 다음 동작을 트리거해야 합니다.

```typescript
// src/lib/prompt-generator.ts
export class PromptGenerator {
  generateIncompleteTaskPrompt(
    incompleteTasks: TaskDetail[],
    summary: { pending: number; inProgress: number; completed: number }
  ): string {
    const lines: string[] = [];
    
    lines.push('⚠️ **작업 완료 알림**');
    lines.push('');
    lines.push('현재 세션에 완료되지 않은 작업이 있습니다.');
    lines.push(`- 🔄 진행 중: ${summary.inProgress}개`);
    lines.push(`- ⏳ 대기 중: ${summary.pending}개`);
    lines.push(`- ✅ 완료됨: ${summary.completed}개`);
    lines.push('');
    lines.push('**남은 작업 목록:**');
    lines.push('');

    for (const task of incompleteTasks) {
      const emoji = task.status === 'in_progress' ? '🔄' : '⏳';
      lines.push(`${emoji} **${task.id}**. ${task.title} (${task.status === 'in_progress' ? '진행 중' : '대기 중'})`);
    }

    lines.push('');
    lines.push('📌 **이 작업들을 완료해주세요.**');
    lines.push('완료 후 `tasks` 도구를 사용하여 상태를 업데이트하세요.');

    return lines.join('\n');
  }
}
```

**⚠️ 중요**: 이 프롬프트는 단순히 반환되는 것이 아니라, **OpenCode API를 통해 사용자 입력처럼 세션으로 직접 전송**되어야 합니다. 이를 통해 에이전트가 "사용자의 요청"으로 인식하고 다음 동작을 수행하게 됩니다.

### 3.5 플러그인 인덱스 수정

```typescript
// src/index.ts 수정 내용

// 세션 종료 체크 함수
async function checkSessionCompletion(
  sessionId: string, 
  client: any  // OpenCode client API
): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();
  const checker = new CompletionChecker(storage, parser);
  const promptGen = new PromptGenerator();

  const result = await checker.checkIncompleteTasks(sessionId);
  
  if (result.hasIncomplete) {
    const prompt = promptGen.generateIncompleteTaskPrompt(
      result.incompleteTasks, 
      result.summary
    );
    
    // ⚠️ 중요: 프롬프트를 사용자 입력처럼 세션으로 직접 전송
    // OpenCode client API를 통해 사용자 메시지로 주입
    await client.session.prompt(sessionId, {
      role: 'user',
      content: prompt
    });
    
    // 또는 사용 가능한 API에 따라:
    // await client.sendMessage(sessionId, prompt);
    // 또는
    // await client.injectUserMessage(sessionId, prompt);
  }
}
```

---

## 4. 데이터 모델 및 인터페이스

### 4.1 기존 타입 (변경 없음)

```typescript
// types/index.ts - 기존 TaskStatus 유지
type TaskStatus = 'pending' | 'in_progress' | 'completed';
```

### 4.2 새로운 타입 추가

```typescript
// types/index.ts 추가

export interface CompletionCheckResult {
  hasIncomplete: boolean;
  incompleteTasks: TaskDetail[];
  summary: {
    pending: number;
    inProgress: number;
    completed: number;
  };
  prompt?: string; // 프롬프트가 생성된 경우
}

export interface SessionEndHandler {
  onSessionEnd(sessionId: string): Promise<string | null>; // 프롬프트 반환 또는 null
}
```

---

## 5. 테스트 전략

### 5.1 버그 수정 테스트

1. **단위 테스트**: Parser의 상태 파싱/생성
   - `[~]` → `in_progress`
   - `[x]` → `completed`
   - `[ ]` → `pending`

2. **통합 테스트**: Update 명령 end-to-end
   - update → 파일 저장 → 파싱 → 상태 확인

### 5.2 세션 종료 체크 테스트

1. **CompletionChecker 단위 테스트**
   - 미완료 task 식별
   - 빈 세션 처리
   - 다중 task 목록 처리

2. **PromptGenerator 단위 테스트**
   - 프롬프트 형식 검증
   - 한국어 메시지 확인

3. **통합 테스트**
   - 세션 종료 시나리오 시뮬레이션

---

## 6. 구현 순서

1. **Phase 1**: 버그 수정
   - parser.ts 수정 (parseTaskList, formatTask)
   - update.ts 검증 (필요시 수정)
   - 테스트 작성 및 실행

2. **Phase 2**: 세션 종료 체크 기능
   - CompletionChecker 구현
   - PromptGenerator 구현
   - index.ts에 이벤트 핸들러 추가 (또는 대안 구현)
   - 테스트 작성 및 실행

3. **Phase 3**: 통합 및 검증
   - 통합 테스트
   - 문서 업데이트
   - 코드 리뷰

---

## 7. 트레이드오프 및 결정사항

### 결정 1: in_progress 상태 표현 방식

**선택**: `[~]` 기호 사용
- **이유**: 마크다운에서 시각적으로 진행 중을 표현하기 좋음
- **대안**: `[ ]` 텍스트 표시 - 하위 호환성은 좋지만 시각적 구분이 어려움

### 결정 2: 세션 종료 이벤트 처리 방식

**선택**: OpenCode 플러그인 이벤트 시스템 사용 (`session.idle` 이벤트 구독)
- **이유**: 가장 자연스러운 UX 제공, OpenCode가 `session.idle` 이벤트를 지원함을 확인
- **구현**: `session.idle` 이벤트 발생 시 미완료 task 체크 및 프롬프트 전송
- **대안**: 명시적 체크 API - 사용자가 직접 호출해야 함 (기본값)

### 결정 3: in_progress 마크다운 표현 방식

**선택**: `[~]` 기호 사용
- **이유**: 시각적으로 진행 중임을 명확히 표현, 마크다운에서 구분하기 쉬움
- **구현**: 
  - `completed` → `[x]`
  - `in_progress` → `[~]`
  - `pending` → `[ ]`
- **하위 호환성**: 기존 파일의 `[ ]`는 `pending`으로 해석

### 결정 4: 하위 호환성

**선택**: 100% 하위 호환성 유지
- **이유**: 기존 task 파일은 `[ ]`로 pending 상태로 해석
- **대안**: 마이그레이션 스크립트 - 복잡성 증가

### 결정 5: 프롬프트 메시지 스타일

**선택**: 경고 스타일 (⚠️)
- **이유**: 미완료 task의 중요성을 강조하여 작업 완료율 향상
- **구현**: "⚠️ **작업 완료 알림**" 헤더 사용
- **대안**: 안내/알림 스타일 - 부드러운 느낌

---

## 8. 참고 문서

- `proposal.md` - 제안서
- `tasks.md` - 작업 목록
- `README.md` - Tasks 플러그인 문서
