# OpenCode Plugin API Reference

> **참고**: 이 문서는 정적 참조용입니다. 최신 정보는 다음 URL에서 확인하세요:
> - https://opencode.ai/docs/plugins/
> - https://opencode.ai/docs/sdk/

## Plugin 함수 시그니처

```typescript
type Plugin = (ctx: PluginContext) => Promise<PluginHooks> | PluginHooks

interface PluginContext {
  project: ProjectInfo      // 현재 프로젝트 정보
  directory: string         // 현재 작업 디렉토리
  worktree: string          // git worktree 경로
  client: OpenCodeClient    // OpenCode SDK 클라이언트
  $: BunShell              // Bun의 shell API
}

interface ProjectInfo {
  name: string
  path: string
  config: Config
}
```

## PluginHooks 인터페이스

```typescript
interface PluginHooks {
  // Command Events
  "command.executed"?: (event: CommandEvent) => Promise<void> | void
  
  // File Events
  "file.edited"?: (event: FileEvent) => Promise<void> | void
  "file.watcher.updated"?: (event: FileWatcherEvent) => Promise<void> | void
  
  // Installation Events
  "installation.updated"?: (event: InstallationEvent) => Promise<void> | void
  
  // LSP Events
  "lsp.client.diagnostics"?: (event: LSPDiagnosticsEvent) => Promise<void> | void
  "lsp.updated"?: (event: LSPUpdatedEvent) => Promise<void> | void
  
  // Message Events
  "message.part.removed"?: (event: MessagePartEvent) => Promise<void> | void
  "message.part.updated"?: (event: MessagePartEvent) => Promise<void> | void
  "message.removed"?: (event: MessageEvent) => Promise<void> | void
  "message.updated"?: (event: MessageEvent) => Promise<void> | void
  
  // Permission Events
  "permission.asked"?: (event: PermissionEvent) => Promise<void> | void
  "permission.replied"?: (event: PermissionReplyEvent) => Promise<void> | void
  
  // Server Events
  "server.connected"?: (event: ServerEvent) => Promise<void> | void
  
  // Session Events
  "session.created"?: (event: SessionEvent) => Promise<void> | void
  "session.compacted"?: (event: SessionEvent) => Promise<void> | void
  "session.deleted"?: (event: SessionEvent) => Promise<void> | void
  "session.diff"?: (event: SessionDiffEvent) => Promise<void> | void
  "session.error"?: (event: SessionErrorEvent) => Promise<void> | void
  "session.idle"?: (event: SessionEvent) => Promise<void> | void
  "session.status"?: (event: SessionStatusEvent) => Promise<void> | void
  "session.updated"?: (event: SessionEvent) => Promise<void> | void
  
  // Todo Events
  "todo.updated"?: (event: TodoEvent) => Promise<void> | void
  
  // Tool Events
  "tool.execute.after"?: (input: ToolInput, output: ToolOutput) => Promise<void> | void
  "tool.execute.before"?: (input: ToolInput, output: ToolOutput) => Promise<void> | void
  
  // TUI Events
  "tui.prompt.append"?: (event: TUIPromptEvent) => Promise<void> | void
  "tui.command.execute"?: (event: TUICommandEvent) => Promise<void> | void
  "tui.toast.show"?: (event: TUIToastEvent) => Promise<void> | void
  
  // Custom Tools
  "tool"?: Record<string, ToolDefinition>
}
```

## Tool 생성 API

```typescript
import { tool } from "@opencode-ai/plugin"

const myTool = tool({
  description: "도구 설명",
  args: {
    // Zod 스키마 기반 인자 정의
    param1: tool.schema.string(),
    param2: tool.schema.number().optional(),
    param3: tool.schema.boolean().default(false),
    param4: tool.schema.enum(["option1", "option2"]),
    param5: tool.schema.array(tool.schema.string())
  },
  async execute(args, context) {
    // args.param1, args.param2 등으로 접근
    // context.directory, context.worktree 등 사용 가능
    return "결과 문자열"
  }
})
```

## Client API

```typescript
// 로깅
await client.app.log({
  service: "my-plugin",
  level: "info",  // "debug" | "info" | "warn" | "error"
  message: "로그 메시지",
  extra: { key: "value" }  // 선택적 추가 데이터
})

// 세션 프롬프트
await client.session.prompt({
  path: { id: sessionID },
  body: {
    parts: [{ type: "text", text: "프롬프트 내용" }],
    noReply: true  // true면 에이전트 응답 없음
  }
})
```

## Bun Shell API ($)

```typescript
// 명령어 실행
await $`echo "Hello World"`

// 결과 캡처
const result = await $`cat file.txt`.text()

// 파이프라인
await $`cat file.txt | grep "pattern"`

// 환경 변수
await $`echo $HOME`

// 템플릿 리터럴
const filename = "test.txt"
await $`cat ${filename}`
```

## Context7에서 최신 API 조회

```
libraryId: /malhashemi/opencode-skills
query: plugin API, hooks, tool definition
```
