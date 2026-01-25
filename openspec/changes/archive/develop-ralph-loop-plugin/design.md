# Design: Ralph Loop Plugin

## 아키텍처 (Architecture)
본 플러그인은 OpenCode Plugin SDK를 기반으로 하며, `Hooks` 인터페이스를 통해 세션 이벤트를 구독하고 SDK Client를 사용하여 세션을 제어합니다.

### 핵심 컴포넌트
1. **LoopManager**: 세션별 반복 횟수 및 상태를 관리하는 싱글톤 클래스.
2. **EventObserver**: `session.idle` 이벤트를 수신하여 LoopManager에 전달.
3. **ActionRunner**: 요약, 파일 저장, 세션 초기화 및 프롬프트 주입을 실행.

## 데이터 모델 (Data Model)
### Plugin Config (`opencode.json`)
```json
{
  "plugin": ["ralph-loop"],
  "ralph-loop": {
    "promiseWord": "DONE",
    "maxRetries": 5,
    "summaryPath": "./.opencode/sessions/",
    "autoRestart": true
  }
}
```

### Internal State
```typescript
interface SessionState {
  retryCount: number;
  lastStatus: string;
}
const sessionRegistry = new Map<string, SessionState>();
```

## 워크플로우 (Workflow)
0. **초기화 (Prompt Injection)**: `chat.message` 훅을 통해 세션의 첫 번째 사용자 메시지를 가로채어, 끝부분에 "작업 완료 시 반드시 Promise Word를 포함하라"는 지시사항을 자동으로 덧붙여 에이전트에게 전달.
1. **감지**: `event` 훅에서 `session.idle` 이벤트 수신.
2. **검증**: `client.session.messages`를 호출하여 마지막 메시지 확인.
3. **분기**:
   - Promise Word 존재 시: 루프 종료, 상태 초기화.
   - Promise Word 부재 시:
     - `retryCount < maxRetries`:
       1. `client.session.summarize` 호출.
       2. 요약 내용을 `.opencode/sessions/(session-id)/ralph_summary.md` 경로에 저장.
       3. `client.session.delete` 및 `create`.
       4. `client.session.prompt`로 재시작 프롬프트 전송 (이때도 Promise Word 지시사항 포함).
       5. `retryCount++`.
     - `retryCount >= maxRetries`: 사용자에게 알림(Toast) 후 중단.

## 기술적 결정 (Technical Decisions)
- **API 및 이벤트 실재 여부 검증**: OpenCode SDK(`@opencode-ai/sdk`)의 타입 정의를 바탕으로 다음 요소들의 사용 가능성을 확인하였습니다.
  - **훅**: `chat.message` (메시지 변조), `event` (상태 감지)
  - **이벤트**: `session.idle` (SDK 413라인)
  - **메서드**: `client.session.messages` (이력 조회), `client.session.summarize` (요약), `client.session.prompt` (명령 주입)
  - **참조**: [OpenCode SDK Type Definitions](https://github.com/opencode-ai/sdk/blob/main/src/gen/types.gen.ts)
- **세션 초기화 방식**: `delete` 후 `create` 방식이 가장 확실한 컨텍스트 초기화를 보장하므로 이를 기본으로 채택.
- **파일 저장**: `node:fs`를 사용하여 `.opencode/sessions/(session-id)/ralph_summary.md` 경로에 마크다운 형식으로 저장. 세션별로 독립된 디렉토리를 생성하여 관리의 편의성을 높임.
- **프롬프트 인젝션 구현**: `chat.message` 훅에서 `output.message.parts`를 수정하여 사용자의 첫 프롬프트에 지시사항을 결합. 이는 시스템 프롬프트 변경보다 모델의 지시 준수율을 높이는 데 유리함.
- **이벤트 수신**: `Hooks.event`를 사용하여 비동기적으로 처리하여 사용자 UI 블로킹 방지.
