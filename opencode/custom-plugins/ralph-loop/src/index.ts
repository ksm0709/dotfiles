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
  isActive: boolean; // ralph-loop이 이 세션에서 활성화되었는지 여부
  processedMessageIds: Set<string>;
}

interface MessagePart {
  id?: string;
  type: string;
  text?: string;
  sessionID?: string;
  messageID?: string;
  [key: string]: unknown;
}

interface RalphMessage {
  id?: string;
  role?: string;
  sessionID?: string;
  parts?: MessagePart[];
}

interface MessageEventPayload {
  message?: RalphMessage;
  part?: MessagePart;
  session?: { id?: string };
}

interface SessionIdleEvent {
  type: "session.idle";
  sessionId: string;
}

interface ChatMessageOutput {
  sessionId?: string;
  message?: { id?: string; sessionID?: string };
  parts: MessagePart[];
}

export const RalphLoopPlugin: Plugin = async (ctx) => {
  let config = ConfigSchema.parse({});
  const { directory, client } = ctx;
  const sessionRegistry = new Map<string, SessionState>();

  // ralph-loop.json 파일에서 설정을 로드합니다.
  const loadConfig = async () => {
    const configPaths = [
      path.join(directory, ".opencode", "ralph-loop.json"),
      path.join(process.env.HOME || "", ".config", "opencode", "ralph-loop.json"),
      path.join(
        process.env.HOME || "",
        ".config",
        "opencode",
        "plugin",
        "ralph-loop",
        "ralph-loop.json",
      ),
    ];

    for (const configPath of configPaths) {
      try {
        const configContent = await fs.readFile(configPath, "utf8");
        const parsed = JSON.parse(configContent);
        config = ConfigSchema.parse(parsed);
        return; // 첫 번째로 발견된 설정 파일을 사용하고 종료
      } catch {
        continue;
      }
    }
  };

  await loadConfig();

  const getOrInitState = (sessionId: string): SessionState => {
    if (!sessionRegistry.has(sessionId)) {
      sessionRegistry.set(sessionId, {
        retryCount: 0,
        messageCount: 0,
        isActive: false,
        processedMessageIds: new Set<string>(),
      });
    }

    const state = sessionRegistry.get(sessionId)!;
    if (!state.processedMessageIds) {
      state.processedMessageIds = new Set<string>();
    }
    return state;
  };

  const instructionText = () =>
    `\n\n[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 '${config.promiseWord}'을 출력하세요.`;

  const createInstructionPart = (sessionId: string, messageId: string): MessagePart => ({
    type: "text",
    text: instructionText(),
    id: `instruction-${Date.now()}`,
    sessionID: sessionId,
    messageID: messageId,
  });

  const extractTextFromMessage = (message: RalphMessage, part?: MessagePart) => {
    const texts: string[] = [];
    if (Array.isArray(message?.parts)) {
      for (const messagePart of message.parts) {
        if (messagePart?.type === "text" && typeof messagePart.text === "string") {
          texts.push(messagePart.text);
        }
      }
    }

    if (part?.type === "text" && typeof part.text === "string") {
      texts.push(part.text);
    }

    return texts
      .map((t) => t ?? "")
      .join("\n")
      .trim();
  };

  const handleKeywordActivation = async (event: MessageEventPayload) => {
    const message = event?.message;
    const sessionId = event?.session?.id ?? message?.sessionID ?? event?.part?.sessionID;
    const rawMessageId = message?.id ?? event?.part?.messageID;

    if (!message || !sessionId || !rawMessageId) return;
    if (message.role !== "user") return;

    const combinedText = extractTextFromMessage(message, event?.part);
    if (!combinedText) return;

    if (!combinedText.toLowerCase().includes("ralph")) return;

    const state = getOrInitState(sessionId);
    const messageId = String(rawMessageId);
    if (state.processedMessageIds.has(messageId)) return;

    state.isActive = true;

    if (!Array.isArray(message.parts)) {
      message.parts = [];
    }

    message.parts.push(createInstructionPart(sessionId, messageId));
    state.processedMessageIds.add(messageId);
  };

  const isSessionIdleEvent = (evt: unknown): evt is SessionIdleEvent => {
    if (typeof evt !== "object" || evt === null) return false;
    const maybeEvent = evt as { type?: unknown; sessionId?: unknown };
    return maybeEvent.type === "session.idle" && typeof maybeEvent.sessionId === "string";
  };

  return {
    config: async (newConfig) => {
      config = ConfigSchema.parse(newConfig);
    },
    event: async ({ event }) => {
      if (!isSessionIdleEvent(event)) return;

      const sessionId = event.sessionId;
      const state = getOrInitState(sessionId);

      // ralph-loop이 활성화되지 않은 세션은 무시
      if (!state.isActive) return;

      const { promiseWord, maxRetries, summaryPath } = config;

      // 1. 마지막 메시지 확인
      const messagesResponse = await client.session.messages({ path: { id: sessionId } });

      if (messagesResponse.error || !messagesResponse.data) return;

      const messages = messagesResponse.data;
      const lastMessage = messages[messages.length - 1];

      if (!lastMessage) return;

      const lastContent = lastMessage.parts.find((p) => p.type === "text")?.text || "";

      // Promise Word 포함 여부 확인
      if (lastContent.includes(promiseWord)) {
        sessionRegistry.delete(sessionId);
        return;
      }

      // 2. 재시도 횟수 확인
      if (state.retryCount >= maxRetries) {
        // Toast 기능이 없을 경우 콘솔 출력으로 대체
        console.warn(
          `[Ralph Loop] 최대 재시도 횟수에 도달했습니다 (${maxRetries}). 루프를 중단합니다.`,
        );
        sessionRegistry.delete(sessionId);
        return;
      }

      // 3. 루프 실행
      state.retryCount++;

      // 요약 생성
      const summaryResponse = await client.session.summarize({ path: { id: sessionId } });

      if (summaryResponse.error || !summaryResponse.data) return;
      const summary = summaryResponse.data;

      // 요약 저장 (디렉토리 생성 포함)
      const sessionSummaryDir = path.join(directory, summaryPath, sessionId);
      await fs.mkdir(sessionSummaryDir, { recursive: true });
      const filePath = path.join(sessionSummaryDir, "ralph_summary.md");

      await fs.writeFile(filePath, typeof summary === "string" ? summary : String(summary), "utf8");

      // 세션 재시작
      await client.session.delete({ path: { id: sessionId } });
      const newSessionResponse = await client.session.create({});

      if (newSessionResponse.error || !newSessionResponse.data) return;
      const newSession = newSessionResponse.data;

      // 상태 이관 (retryCount 승계, 활성화 상태 유지)
      sessionRegistry.set(newSession.id, {
        retryCount: state.retryCount,
        messageCount: 0,
        skipInstruction: true,
        isActive: true, // 새 세션에서도 ralph-loop 활성화 상태 유지
        processedMessageIds: new Set<string>(),
      });
      sessionRegistry.delete(sessionId);

      const restartPrompt = `이전 세션의 요약 내용입니다:\n\n${summary}\n\n위 내용을 바탕으로 작업을 계속해 주세요.\n\n[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 "${promiseWord}"를 출력하세요.`;

      await client.session.prompt({
        path: { id: newSession.id },
        body: { parts: [{ type: "text", text: restartPrompt }] },
      });
    },
    "chat.message": async (input: unknown, output: ChatMessageOutput) => {
      const sessionId = output.sessionId || output.message?.sessionID;
      if (!sessionId) return;

      const state = getOrInitState(sessionId);

      state.messageCount++;

      // 사용자 입력 메시지에서 'ralph' 키워드 확인 (대소문자 무관)
      const userInputText = (typeof input === "string" ? input : String(input)).toLowerCase();
      const containsRalphKeyword = userInputText.includes("ralph");

      // 오직 'ralph' 키워드가 포함되어 있을 때만 프롬프트 인젝션 및 세션 활성화
      if (containsRalphKeyword) {
        // 이 세션에서 ralph-loop을 활성화
        state.isActive = true;
        output.parts.push(createInstructionPart(sessionId, output.message?.id || "unknown"));
      }
    },
    "message.updated": async (eventPayload: MessageEventPayload) => {
      await handleKeywordActivation(eventPayload);
    },
    "message.part.updated": async (eventPayload: MessageEventPayload) => {
      await handleKeywordActivation(eventPayload);
    },
  };
};

export default RalphLoopPlugin;
