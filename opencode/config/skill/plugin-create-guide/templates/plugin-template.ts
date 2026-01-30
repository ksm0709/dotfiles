import type { Plugin } from "@opencode-ai/plugin"
import { tool } from "@opencode-ai/plugin"

/**
 * {{PLUGIN_NAME}}
 * 
 * {{PLUGIN_DESCRIPTION}}
 * 
 * @see https://opencode.ai/docs/plugins/
 */

export const {{PLUGIN_NAME}}: Plugin = async ({
  project,      // 현재 프로젝트 정보
  directory,    // 현재 작업 디렉토리
  worktree,     // git worktree 경로
  client,       // OpenCode SDK 클라이언트
  $             // Bun shell API
}) => {
  // 플러그인 초기화 로그
  await client.app.log({
    service: "{{PLUGIN_SERVICE_NAME}}",
    level: "info",
    message: "Plugin initialized",
    extra: { directory, worktree }
  })

  return {
    // 세션 완료 시 처리
    "session.idle": async (event) => {
      await client.app.log({
        service: "{{PLUGIN_SERVICE_NAME}}",
        level: "info",
        message: "Session completed"
      })
    },

    // 도구 실행 전 처리
    "tool.execute.before": async (input, output) => {
      // 예: 특정 파일 접근 차단
      if (input.tool === "read" && output.args.filePath?.includes(".env")) {
        throw new Error("Access to .env files is not allowed")
      }
    },

    // 도구 실행 후 처리
    "tool.execute.after": async (input, output) => {
      // 예: 실행 결과 로깅
      await client.app.log({
        service: "{{PLUGIN_SERVICE_NAME}}",
        level: "debug",
        message: `Tool ${input.tool} executed`,
        extra: { input, output }
      })
    }
  }
}

/**
 * Custom Tool 예제 (선택사항)
 * 
 * Custom tool을 추가하려면 hooks 객체에 tool 속성을 추가하세요.
 */
export const {{PLUGIN_NAME}}WithTools: Plugin = async (ctx) => {
  return {
    tool: {
      {{TOOL_NAME}}: tool({
        description: "{{TOOL_DESCRIPTION}}",
        args: {
          param1: tool.schema.string(),
          param2: tool.schema.number().optional(),
          param3: tool.schema.boolean().default(false)
        },
        async execute(args, context) {
          const { directory, worktree } = context
          
          // 도구 로직 구현
          const result = `Executed with param1=${args.param1}`
          
          return result
        }
      })
    }
  }
}
