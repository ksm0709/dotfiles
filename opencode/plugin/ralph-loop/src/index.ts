import type { Plugin } from "@opencode-ai/plugin";
import { z } from "zod";
import * as path from "node:path";
import * as fs from "node:fs/promises";

const ConfigSchema = z.object({
  promiseWord: z.string().default("DONE"),
  maxRetries: z.number().default(5),
  summaryPath: z.string().default("./.opencode/sessions/"),
  autoRestart: z.boolean().default(true),
});

interface SessionState {
  retryCount: number;
  messageCount: number;
  skipInstruction?: boolean;
}

export const RalphLoopPlugin: Plugin = async ({ directory, client }) => {
  let config = ConfigSchema.parse({});
  const sessionRegistry = new Map<string, SessionState>();

  // ralph-loop.json 파일에서 설정을 로드합니다.
  const loadConfig = async () => {
    try {
      // 1. 프로젝트 루트 확인
      const projectConfigPath = path.join(directory, "ralph-loop.json");
      // 2. 플러그인 설치 경로 확인 (전역 설정용)
      const globalConfigPath = path.join(
        process.env.HOME || "",
        ".config/opencode/plugin/ralph-loop/ralph-loop.json",
      );

      let configContent = "{}";
      try {
        configContent = await fs.readFile(projectConfigPath, "utf8");
      } catch {
        try {
          configContent = await fs.readFile(globalConfigPath, "utf8");
        } catch {
          // 설정 파일이 없으면 기본값 사용
        }
      }

      const parsed = JSON.parse(configContent);
      config = ConfigSchema.parse(parsed);
    } catch {
      // 파싱 에러 시 기본값 사용
    }
  };

  await loadConfig();

  const getOrInitState = (sessionId: string): SessionState => {
    if (!sessionRegistry.has(sessionId)) {
      sessionRegistry.set(sessionId, { retryCount: 0, messageCount: 0 });
    }
    return sessionRegistry.get(sessionId)!;
  };

  return {
    config: async (newConfig) => {
      config = ConfigSchema.parse(newConfig);
    },
    event: async ({ event }) => {
      if (event.type !== "session.idle") return;

      const sessionId = event.data.sessionId;
      const state = getOrInitState(sessionId);
      const { promiseWord, maxRetries, summaryPath } = config;

      // 1. 마지막 메시지 확인
      const messages = await client.session.messages(sessionId);
      const lastMessage = messages[messages.length - 1];

      if (!lastMessage || lastMessage.role !== "assistant") return;

      const lastContent = lastMessage.parts.find((p) => p.type === "text")?.text || "";

      // Promise Word 포함 여부 확인
      if (lastContent.includes(promiseWord)) {
        sessionRegistry.delete(sessionId);
        return;
      }

      // 2. 재시도 횟수 확인
      if (state.retryCount >= maxRetries) {
        await client.toast(
          `[Ralph Loop] 최대 재시도 횟수에 도달했습니다 (${maxRetries}). 루프를 중단합니다.`,
        );
        sessionRegistry.delete(sessionId);
        return;
      }

      // 3. 루프 실행
      state.retryCount++;

      // 요약 생성
      const summary = await client.session.summarize(sessionId);

      // 요약 저장 (디렉토리 생성 포함)
      // directory는 문자열 경로이므로 fs를 사용하여 저장
      const sessionSummaryDir = path.join(directory, summaryPath, sessionId);
      await fs.mkdir(sessionSummaryDir, { recursive: true });
      const filePath = path.join(sessionSummaryDir, "ralph_summary.md");

      await fs.writeFile(filePath, summary, "utf8");

      // 세션 재시작
      await client.session.delete(sessionId);
      const newSession = await client.session.create();

      // 상태 이관 (retryCount 승계, 중복 주입 방지 플래그 설정)
      sessionRegistry.set(newSession.id, {
        retryCount: state.retryCount,
        messageCount: 0,
        skipInstruction: true,
      });
      sessionRegistry.delete(sessionId);

      const restartPrompt = `이전 세션의 요약 내용입니다:\n\n${summary}\n\n위 내용을 바탕으로 작업을 계속해 주세요.\n(중요: 모든 작업이 완료되면 반드시 "${promiseWord}"를 출력하세요.)`;

      await client.session.prompt(newSession.id, restartPrompt);
    },
    "chat.message": async (input, output) => {
      const sessionId = output.sessionId;
      const state = getOrInitState(sessionId);

      state.messageCount++;

      // 첫 번째 메시지이고 지시사항 건너뛰기 플래그가 없을 때만 지시사항 추가
      if (state.messageCount === 1 && !state.skipInstruction) {
        const instruction = `\n\n(중요: 모든 작업이 완료되면 반드시 '${config.promiseWord}'을 출력하세요.)`;
        output.parts.push({
          type: "text",
          text: instruction,
        });
      }
    },
  };
};

export default RalphLoopPlugin;
