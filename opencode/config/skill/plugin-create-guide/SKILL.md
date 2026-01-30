---
name: plugin-create-guide
description: OpenCode 플러그인 및 스킬 생성을 위한 종합 가이드. Plugin(JavaScript/TypeScript)과 Skill(Markdown) 두 가지 확장 방식을 모두 지원하며, 최신 API 정보를 실시간으로 제공합니다.
license: MIT
allowed-tools:
  - read
  - write
  - bash
  - webfetch
  - context7_resolve-library-id
  - context7_query-docs
metadata:
  version: "1.0.0"
  author: "OpenCode Assistant"
  last_updated: "2026-01-30"
---

# OpenCode Plugin/Skill 생성 가이드

OpenCode는 두 가지 확장 메커니즘을 제공합니다:

1. **Plugin** - JavaScript/TypeScript로 작성하는 코드 기반 확장
2. **Skill** - Markdown로 작성하는 프롬프트 기반 확장

## 어떤 것을 선택해야 할까요?

### Skill을 선택하는 경우
- ✅ AI 에이전트의 동작을 가이드하고 싶을 때
- ✅ 특정 워크플로우나 프로세스를 정의하고 싶을 때
- ✅ 복잡한 코드 작성 없이 빠르게 확장하고 싶을 때
- ✅ 팀 내에서 지식을 공유하고 싶을 때

### Plugin을 선택하는 경우
- ✅ OpenCode의 이벤트에 후킹하여 동작을 수정하고 싶을 때
- ✅ 외부 서비스와 통합하고 싶을 때
- ✅ 새로운 도구(Tool)를 추가하고 싶을 때
- ✅ TypeScript/JavaScript로 복잡한 로직을 구현해야 할 때

---

## Skill 생성하기

### 1. 기본 구조

Skill은 다음 디렉토리 구조를 따릅니다:

```
.opencode/skills/my-skill/
├── SKILL.md              # 필수: 메인 스킬 파일
├── scripts/              # 선택: 실행 스크립트
├── references/           # 선택: 참조 문서
└── assets/               # 선택: 기타 파일
```

### 2. SKILL.md 작성

```markdown
---
name: my-skill                    # 디렉토리명과 일치해야 함
description: 스킬 설명 (20자 이상)  # 필수, 검색에 사용됨
license: MIT                      # 선택
allowed-tools:                    # 선택, 허용할 도구 목록
  - read
  - write
  - bash
metadata:                         # 선택, 사용자 정의 메타데이터
  version: "1.0"
  author: "Your Name"
---

# 스킬 제목

스킬의 상세 설명과 사용 방법을 작성합니다.

## 사용 방법

1. 첫 번째 단계
2. 두 번째 단계
3. 결과 확인

## 참조 파일

- `scripts/helper.sh` - 유틸리티 스크립트
- `references/guide.md` - 상세 가이드
```

### 3. 네이밍 규칙

| 항목 | 규칙 | 예시 |
|------|------|------|
| 디렉토리 | 소문자와 하이픈 | `my-skill`, `code-reviewer` |
| Frontmatter name | 디렉토리명과 일치 | `name: my-skill` |
| 도구명 | 자동 생성 (언더스코어) | `skills_my_skill` |

### 4. Skill 사용하기

Skill 생성 후 OpenCode를 재시작하면 자동으로 등록됩니다:

```
skills_my_skill    # 스킬 호출
```

---

## Plugin 생성하기

### 1. 기본 구조

Plugin은 다음 위치에 생성할 수 있습니다:

- **프로젝트 레벨**: `.opencode/plugins/my-plugin.ts`
- **글로벌**: `~/.config/opencode/plugins/my-plugin.ts`

### 2. 기본 Plugin 템플릿

```typescript
import type { Plugin } from "@opencode-ai/plugin"

export const MyPlugin: Plugin = async ({ 
  project,      // 현재 프로젝트 정보
  directory,    // 현재 작업 디렉토리
  worktree,     // git worktree 경로
  client,       // OpenCode SDK 클라이언트
  $             // Bun shell API
}) => {
  console.log("Plugin initialized!")
  
  return {
    // Hook implementations
    "tool.execute.before": async (input, output) => {
      // 도구 실행 전 처리
    },
    "session.idle": async (event) => {
      // 세션 완료 시 처리
    }
  }
}
```

### 3. 외부 의존성 사용

`.opencode/package.json` 또는 `~/.config/opencode/package.json` 생성:

```json
{
  "dependencies": {
    "some-package": "^1.0.0"
  }
}
```

OpenCode 시작 시 자동으로 `bun install`을 실행합니다.

### 4. Custom Tool 추가

```typescript
import { type Plugin, tool } from "@opencode-ai/plugin"

export const CustomToolsPlugin: Plugin = async (ctx) => {
  return {
    tool: {
      mytool: tool({
        description: "커스텀 도구 설명",
        args: {
          foo: tool.schema.string(),
          bar: tool.schema.number().optional()
        },
        async execute(args, context) {
          const { directory, worktree } = context
          return `Result: ${args.foo}`
        }
      })
    }
  }
}
```

---

## 사용 가능한 이벤트 (Hooks)

### Command Events
- `command.executed`

### File Events
- `file.edited`
- `file.watcher.updated`

### Installation Events
- `installation.updated`

### LSP Events
- `lsp.client.diagnostics`
- `lsp.updated`

### Message Events
- `message.part.removed`
- `message.part.updated`
- `message.removed`
- `message.updated`

### Permission Events
- `permission.asked`
- `permission.replied`

### Server Events
- `server.connected`

### Session Events
- `session.created`
- `session.compacted`
- `session.deleted`
- `session.diff`
- `session.error`
- `session.idle`
- `session.status`
- `session.updated`

### Todo Events
- `todo.updated`

### Tool Events
- `tool.execute.after`
- `tool.execute.before`

### TUI Events
- `tui.prompt.append`
- `tui.command.execute`
- `tui.toast.show`

---

## 최신 정보 조회 방법

### 공식 문서
- **Plugin Docs**: https://opencode.ai/docs/plugins/
- **Skill Docs**: https://opencode.ai/docs/skills/
- **SDK Docs**: https://opencode.ai/docs/sdk/

### Context7 라이브러리 검색
Context7에서 OpenCode 관련 라이브러리를 검색할 수 있습니다:

```
libraryName: opencode
```

주요 라이브러리:
- `/malhashemi/opencode-skills` - Skills 플러그인
- `/shekohex/opencode-pty` - PTY 플러그인
- `/sst/opencode-sdk-js` - JavaScript SDK

### npm 플러그인 검색
커뮤니티 플러그인은 다음에서 찾을 수 있습니다:
- https://opencode.ai/docs/ecosystem#plugins

---

## 예제 모음

### 예제 1: 알림 플러그인

```javascript
export const NotificationPlugin = async ({ $ }) => {
  return {
    "session.idle": async () => {
      await $`osascript -e 'display notification "Session completed!" with title "opencode"'`
    }
  }
}
```

### 예제 2: .env 파일 보호

```javascript
export const EnvProtection = async () => {
  return {
    "tool.execute.before": async (input, output) => {
      if (input.tool === "read" && output.args.filePath.includes(".env")) {
        throw new Error("Do not read .env files")
      }
    }
  }
}
```

### 예제 3: 로깅 플러그인

```typescript
export const LoggingPlugin = async ({ client }) => {
  await client.app.log({
    service: "my-plugin",
    level: "info",
    message: "Plugin initialized",
    extra: { foo: "bar" }
  })
}
```

---

## npm으로 플러그인 배포

### 1. package.json 준비

```json
{
  "name": "opencode-my-plugin",
  "version": "1.0.0",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "peerDependencies": {
    "@opencode-ai/plugin": "^1.0.0"
  }
}
```

### 2. OpenCode 설정에 추가

```json
{
  "plugins": ["opencode-my-plugin"]
}
```

---

## 문제 해결

### Skill이 등록되지 않는 경우
1. OpenCode 재시작
2. SKILL.md 문법 확인 (YAML frontmatter)
3. name이 디렉토리명과 일치하는지 확인
4. description이 20자 이상인지 확인

### Plugin이 로드되지 않는 경우
1. TypeScript 컴파일 오류 확인
2. 의존성 설치 확인 (`bun install`)
3. export 구문 확인
4. OpenCode 로그 확인

### 의존성 문제
- `.opencode/package.json` 또는 `~/.config/opencode/package.json` 확인
- `bun install` 수동 실행
- 캐시 삭제: `~/.cache/opencode/node_modules/`

---

## 참고 자료

- **이 스킬의 템플릿**: `templates/` 디렉토리 참조
- **상세 API 문서**: `references/` 디렉토리 참조
- **최신 문서**: https://opencode.ai/docs/plugins/
- **Context7 라이브러리**: Context7에서 "opencode" 검색
