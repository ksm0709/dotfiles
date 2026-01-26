import { z } from "zod";
import * as path from "node:path";
import * as fs from "node:fs/promises";
const ConfigSchema = z.object({
    promiseWord: z.string().default("DONE"),
    maxRetries: z.number().default(5),
    summaryPath: z.string().default("./.opencode/sessions/"),
    autoRestart: z.boolean().default(true),
});
export const RalphLoopPlugin = async (ctx) => {
    let config = ConfigSchema.parse({});
    const { directory, client } = ctx;
    const sessionRegistry = new Map();
    // ralph-loop.json 파일에서 설정을 로드합니다.
    const loadConfig = async () => {
        const configPaths = [
            path.join(directory, ".opencode", "ralph-loop.json"),
            path.join(process.env.HOME || "", ".config", "opencode", "ralph-loop.json"),
            path.join(process.env.HOME || "", ".config", "opencode", "plugin", "ralph-loop", "ralph-loop.json"),
        ];
        for (const configPath of configPaths) {
            try {
                const configContent = await fs.readFile(configPath, "utf8");
                const parsed = JSON.parse(configContent);
                config = ConfigSchema.parse(parsed);
                return; // 첫 번째로 발견된 설정 파일을 사용하고 종료
            }
            catch {
                continue;
            }
        }
    };
    await loadConfig();
    const getOrInitState = (sessionId) => {
        if (!sessionRegistry.has(sessionId)) {
            sessionRegistry.set(sessionId, { retryCount: 0, messageCount: 0 });
        }
        return sessionRegistry.get(sessionId);
    };
    return {
        config: async (newConfig) => {
            config = ConfigSchema.parse(newConfig);
        },
        event: async ({ event }) => {
            if (event.type !== "session.idle")
                return;
            const sessionId = event.sessionId;
            const state = getOrInitState(sessionId);
            const { promiseWord, maxRetries, summaryPath } = config;
            // 1. 마지막 메시지 확인
            const messagesResponse = await client.session.messages({ path: { id: sessionId } });
            if (messagesResponse.error || !messagesResponse.data)
                return;
            const messages = messagesResponse.data;
            const lastMessage = messages[messages.length - 1];
            if (!lastMessage)
                return;
            const lastContent = lastMessage.parts.find((p) => p.type === "text")?.text || "";
            // Promise Word 포함 여부 확인
            if (lastContent.includes(promiseWord)) {
                sessionRegistry.delete(sessionId);
                return;
            }
            // 2. 재시도 횟수 확인
            if (state.retryCount >= maxRetries) {
                // Toast 기능이 없을 경우 콘솔 출력으로 대체
                console.warn(`[Ralph Loop] 최대 재시도 횟수에 도달했습니다 (${maxRetries}). 루프를 중단합니다.`);
                sessionRegistry.delete(sessionId);
                return;
            }
            // 3. 루프 실행
            state.retryCount++;
            // 요약 생성
            const summaryResponse = await client.session.summarize({ path: { id: sessionId } });
            if (summaryResponse.error || !summaryResponse.data)
                return;
            const summary = summaryResponse.data;
            // 요약 저장 (디렉토리 생성 포함)
            const sessionSummaryDir = path.join(directory, summaryPath, sessionId);
            await fs.mkdir(sessionSummaryDir, { recursive: true });
            const filePath = path.join(sessionSummaryDir, "ralph_summary.md");
            await fs.writeFile(filePath, typeof summary === 'string' ? summary : String(summary), "utf8");
            // 세션 재시작
            await client.session.delete({ path: { id: sessionId } });
            const newSessionResponse = await client.session.create({});
            if (newSessionResponse.error || !newSessionResponse.data)
                return;
            const newSession = newSessionResponse.data;
            // 상태 이관 (retryCount 승계, 중복 주입 방지 플래그 설정)
            sessionRegistry.set(newSession.id, {
                retryCount: state.retryCount,
                messageCount: 0,
                skipInstruction: true,
            });
            sessionRegistry.delete(sessionId);
            const restartPrompt = `이전 세션의 요약 내용입니다:\n\n${summary}\n\n위 내용을 바탕으로 작업을 계속해 주세요.\n\n[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 "${promiseWord}"를 출력하세요.`;
            await client.session.prompt({
                path: { id: newSession.id },
                body: { parts: [{ type: "text", text: restartPrompt }] }
            });
        },
        "chat.message": async (input, output) => {
            const sessionId = output.sessionId || output.message?.sessionID;
            if (!sessionId)
                return;
            const state = getOrInitState(sessionId);
            state.messageCount++;
            // 사용자 입력 메시지에서 'ralph' 키워드 확인 (대소문자 무관)
            const userInputText = (typeof input === 'string' ? input : String(input)).toLowerCase();
            const containsRalphKeyword = userInputText.includes('ralph');
            // 프롬프트 인젝션 조건:
            // 1. 첫 번째 메시지이고 지시사항 건너뛰기 플래그가 없을 때
            // 2. 또는 메시지에 'ralph' 키워드가 포함되어 있을 때
            const shouldInjectInstruction = ((state.messageCount === 1 && !state.skipInstruction) ||
                containsRalphKeyword);
            if (shouldInjectInstruction) {
                const instruction = `\n\n[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 '${config.promiseWord}'을 출력하세요.`;
                output.parts.push({
                    type: "text",
                    text: instruction,
                    id: `instruction-${Date.now()}`,
                    sessionID: sessionId,
                    messageID: output.message?.id || 'unknown',
                });
            }
        },
    };
};
export default RalphLoopPlugin;
