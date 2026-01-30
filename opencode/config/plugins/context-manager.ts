/**
 * Context Manager Plugin
 *
 * Plugin for automatic 2-layer context management for agents.
 *
 * Phase 1: Semantic Memory 기록 구조 개선
 * - Task 1.4: 전용 메타데이터 도구 (context_intent, context_decision, context_learning)
 * - Task 1.5: 프롬프트 인젝션 (context_start 반환값에 지침 추가)
 * - Task 1.5.1: 리마인더 주입 (도구 미사용 시 알림)
 * - Task 1.6: Intent 추출 로직 (규칙 기반 폴백)
 * - Task 1.7: Outcome 추출 로직
 * - Task 1.9: SemanticRecord 전송
 */

import type { Plugin } from "@opencode-ai/plugin";
import { tool } from "@opencode-ai/plugin";
import * as fs from "node:fs";
import * as path from "node:path";
import * as os from "node:os";

// ==================== DEBUG LOGGING ====================
const DEBUG = true;
const LOG_FILE = path.join(
  os.homedir(),
  ".local/share/opencode/log/context-manager-debug.log",
);

function debugLog(message: string, data?: any) {
  if (!DEBUG) return;
  const timestamp = new Date().toISOString();
  const logEntry = `[${timestamp}] ${message}${data ? `: ${JSON.stringify(data, null, 2)}` : ""}\n`;
  try {
    fs.appendFileSync(LOG_FILE, logEntry);
  } catch (e) {
    // ignore write errors
  }
}
// ========================================================

// ==================== TYPES ====================
interface IntentCapture {
  intent: string;
  source: "agent" | "inferred" | "unknown";
  confidence: number;
}

interface SemanticRecordPayload {
  session_id: string;
  intent: string;
  action: string;
  outcome: string;
  tool_name: string;
  tool_args: Record<string, any>;
  success: boolean;
  decision?: {
    decision_type: string;
    choice: string;
    alternatives: string[];
    rationale: string;
    is_user_preference?: boolean;
  };
  importance: "low" | "medium" | "high" | "critical";
}

interface MetadataToolState {
  intentCalled: boolean;
  reminderCount: number;
  sessionStarted: boolean;
}

// ==================== PROBLEM TRACKING STATE (Phase 2) ====================
interface ProblemTrackingState {
  activeProblemId: string | null;
  lastErrorOutput: string | null;
}
// ===============================================

// ==================== INTENT EXTRACTION (Task 1.6) ====================
/**
 * 도구 인자에서 Intent 추출 (규칙 기반)
 */
function captureIntent(
  toolName: string,
  args: Record<string, any>,
): IntentCapture {
  let intent = `도구 실행: ${toolName}`;
  let source: "agent" | "inferred" | "unknown" = "inferred";
  let confidence = 0.5;

  switch (toolName) {
    case "bash":
      // bash의 description 필드 활용
      if (args.description) {
        intent = args.description;
        source = "agent"; // description은 에이전트가 제공
        confidence = 0.8;
      } else if (args.command) {
        intent = inferIntentFromCommand(args.command);
        confidence = 0.4;
      }
      break;

    case "read":
      if (args.filePath) {
        const fileName = getFileName(args.filePath);
        intent = `파일 내용 확인: ${fileName}`;
        confidence = 0.6;
      }
      break;

    case "edit":
      if (args.filePath) {
        const fileName = getFileName(args.filePath);
        intent = `파일 수정: ${fileName}`;
        confidence = 0.6;
      }
      break;

    case "write":
      if (args.filePath) {
        const fileName = getFileName(args.filePath);
        intent = `새 파일 작성: ${fileName}`;
        confidence = 0.6;
      }
      break;

    case "task":
      if (args.description) {
        intent = args.description;
        source = "agent";
        confidence = 0.8;
      } else {
        intent = `서브태스크 위임: ${args.subagent_type || "unknown"}`;
        confidence = 0.4;
      }
      break;

    case "question":
      if (args.questions && args.questions[0]?.header) {
        intent = `사용자 의견 수집: ${args.questions[0].header}`;
        confidence = 0.7;
      } else {
        intent = "사용자 의견 수집";
        confidence = 0.3;
      }
      break;

    case "glob":
    case "grep":
      if (args.pattern) {
        intent = `코드베이스 검색: ${args.pattern}`;
        confidence = 0.6;
      }
      break;

    default:
      intent = `도구 실행: ${toolName}`;
      source = "unknown";
      confidence = 0.2;
  }

  return { intent, source, confidence };
}

/**
 * 명령어에서 Intent 추론
 */
function inferIntentFromCommand(command: string): string {
  const cmd = command.toLowerCase();

  if (cmd.includes("pytest") || cmd.includes("test")) {
    return "테스트 실행";
  }
  if (cmd.includes("git commit")) {
    return "변경 사항 커밋";
  }
  if (cmd.includes("git push")) {
    return "원격 저장소에 푸시";
  }
  if (cmd.includes("git pull")) {
    return "원격 저장소에서 풀";
  }
  if (cmd.includes("npm install") || cmd.includes("pip install")) {
    return "패키지 설치";
  }
  if (cmd.includes("npm run") || cmd.includes("npm start")) {
    return "애플리케이션 실행";
  }
  if (cmd.includes("mkdir")) {
    return "디렉토리 생성";
  }
  if (cmd.includes("rm ") || cmd.includes("rm -")) {
    return "파일/디렉토리 삭제";
  }

  // 기본 폴백
  const firstWord = command.split(" ")[0];
  return `명령어 실행: ${firstWord}`;
}

/**
 * 파일 경로에서 파일명 추출
 */
function getFileName(filePath: string): string {
  return path.basename(filePath);
}

// ==================== OUTCOME EXTRACTION (Task 1.7) ====================
/**
 * 도구 실행 결과에서 Outcome 추출
 */
function captureOutcome(
  toolName: string,
  output: string,
  success: boolean,
): string {
  if (!success) {
    return `실패 - ${extractErrorSummary(output)}`;
  }

  switch (toolName) {
    case "bash":
      return summarizeBashOutput(output);

    case "read":
      const lines = output.split("\n").length;
      return `성공 - ${lines}줄 읽음`;

    case "edit":
      return "성공 - 파일 수정 완료";

    case "write":
      return "성공 - 파일 생성 완료";

    case "question":
      const userChoice = extractUserChoice(output);
      return `사용자 응답 수신: ${userChoice}`;

    case "task":
      return summarizeTaskOutput(output);

    case "glob":
    case "grep":
      const matchCount = countMatches(output);
      return `검색 완료 - ${matchCount}개 결과`;

    default:
      return success ? "성공" : "실패";
  }
}

/**
 * 에러 메시지 요약
 */
function extractErrorSummary(output: string): string {
  // 일반적인 에러 패턴 매칭
  const errorPatterns = [
    /Error: (.+)/i,
    /Exception: (.+)/i,
    /Failed: (.+)/i,
    /error\[.+\]: (.+)/i,
  ];

  for (const pattern of errorPatterns) {
    const match = output.match(pattern);
    if (match) {
      return match[1].slice(0, 100);
    }
  }

  // 첫 줄 반환
  const firstLine = output.split("\n")[0];
  return firstLine.slice(0, 100) || "알 수 없는 오류";
}

/**
 * Bash 출력 요약
 */
function summarizeBashOutput(output: string): string {
  const lines = output.split("\n").filter((l) => l.trim());

  if (lines.length === 0) {
    return "성공 - 출력 없음";
  }

  if (lines.length === 1) {
    return `성공 - ${lines[0].slice(0, 80)}`;
  }

  // 테스트 결과 감지
  if (output.includes("passed") && output.includes("test")) {
    const passMatch = output.match(/(\d+) passed/);
    if (passMatch) {
      return `성공 - ${passMatch[1]}개 테스트 통과`;
    }
  }

  // 기본 요약
  return `성공 - ${lines.length}줄 출력`;
}

/**
 * 사용자 응답 추출
 */
function extractUserChoice(output: string): string {
  // question 도구의 응답 형식에서 추출
  const match = output.match(/selected: (.+)/i) || output.match(/chose: (.+)/i);
  if (match) {
    return match[1].slice(0, 50);
  }
  return output.slice(0, 50) || "알 수 없음";
}

/**
 * Task 도구 출력 요약
 */
function summarizeTaskOutput(output: string): string {
  if (output.includes("completed")) {
    return "서브태스크 완료";
  }
  if (output.includes("failed")) {
    return "서브태스크 실패";
  }
  return `서브태스크 실행 - ${output.slice(0, 50)}`;
}

/**
 * 검색 결과 개수
 */
function countMatches(output: string): number {
  const lines = output.split("\n").filter((l) => l.trim());
  return lines.length;
}

function formatJsonOutput(raw: string): string {
  try {
    const parsed = JSON.parse(raw);
    return JSON.stringify(parsed, null, 2);
  } catch (error) {
    return raw;
  }
}

function normalizeAgentThoughts(
  listInput?: string[] | null,
  textInput?: string | null,
): string[] {
  if (Array.isArray(listInput) && listInput.length > 0) {
    return listInput.map((item) => (item ?? "").toString().trim()).filter(Boolean);
  }
  if (textInput && textInput.trim().length > 0) {
    return textInput
      .split("\n")
      .map((line) => line.trim())
      .filter(Boolean);
  }
  return [];
}

function formatTimestamp(value: unknown): string {
  if (typeof value === "number" && Number.isFinite(value)) {
    const date = new Date(value * 1000);
    if (!Number.isNaN(date.getTime())) {
      return date.toISOString();
    }
  }
  if (typeof value === "string" && value.trim().length > 0) {
    return value;
  }
  return "N/A";
}

function formatRelatedMemoriesMarkdown(memories: any): string {
  if (!Array.isArray(memories) || memories.length === 0) {
    return "_관련 기억 없음_";
  }

  return memories
    .map((memory, index) => {
      const rawScore = memory?.score;
      const score =
        typeof rawScore === "number"
          ? rawScore.toFixed(2)
          : typeof rawScore === "string"
            ? rawScore
            : "?";
      const content = (memory?.content ?? "(내용 없음)").toString().trim();
      const truncated = content.length > 200 ? `${content.slice(0, 200)}...` : content;
      const tags = Array.isArray(memory?.tags) && memory.tags.length > 0
        ? ` (Tags: ${memory.tags.join(", ")})`
        : "";
      return `${index + 1}. [${score}] ${truncated}${tags}`;
    })
    .join("\n");
}

function formatContextStartMarkdown(raw: string): string {
  try {
    const data = JSON.parse(raw);
    const lines: string[] = ["## 🧠 Context Briefing"];
    if (typeof data.context_summary === "string" && data.context_summary.trim().length > 0) {
      lines.push(data.context_summary.trim());
    } else {
      lines.push("⚠️ context_summary unavailable. Refer to related memories below.");
    }

    lines.push("", "### 📚 Related Memories");
    lines.push(formatRelatedMemoriesMarkdown(data.relevant_memories));

    lines.push("", "### 🔧 Episode");
    lines.push(`- Episode ID: ${data.episode_id || "N/A"}`);
    lines.push(`- Started: ${formatTimestamp(data.timestamp)}`);
    if (data.task) {
      lines.push(`- Task: ${data.task}`);
    }

    return lines.join("\n");
  } catch (error) {
    return [
      "⚠️ Failed to parse context_start response as JSON.",
      "```json",
      raw,
      "```",
    ].join("\n");
  }
}

/**
 * 성공 여부 판단
 */
function determineSuccess(toolName: string, output: string): boolean {
  const failurePatterns = [
    /Error:/i,
    /Exception:/i,
    /Failed/i,
    /exit code [1-9]/i,
    /ModuleNotFoundError/,
    /ImportError/,
    /FAILED/,
  ];

  for (const pattern of failurePatterns) {
    if (pattern.test(output)) {
      return false;
    }
  }

  return true;
}

// ==================== ERROR DETECTION (Phase 2: Task 2.4) ====================
/**
 * 오류 패턴 목록 (문제 추적용)
 */
const ERROR_PATTERNS = [
  /Error:/i,
  /Exception:/i,
  /Failed/i,
  /exit code [1-9]/i,
  /ModuleNotFoundError/,
  /ImportError/,
  /SyntaxError/i,
  /TypeError/i,
  /ValueError/i,
  /AttributeError/i,
  /FAILED/,
];

/**
 * 오류 메시지 추출
 */
function extractErrorMessage(output: string): string | null {
  for (const pattern of ERROR_PATTERNS) {
    const match = output.match(pattern);
    if (match) {
      // 매치된 라인 추출
      const lines = output.split("\n");
      for (const line of lines) {
        if (pattern.test(line)) {
          return line.slice(0, 300);
        }
      }
      return match[0];
    }
  }
  return null;
}

/**
 * 오류 감지 여부 확인
 */
function detectsError(output: string): boolean {
  for (const pattern of ERROR_PATTERNS) {
    if (pattern.test(output)) {
      return true;
    }
  }
  return false;
}

/**
 * 중요도 판단
 */
function assessImportance(
  toolName: string,
  args: Record<string, any>,
  success: boolean,
): "low" | "medium" | "high" | "critical" {
  // 실패한 작업은 medium 이상
  if (!success) {
    return "medium";
  }

  // 파일 변경은 medium
  if (toolName === "write" || toolName === "edit") {
    return "medium";
  }

  // 커밋, 푸시 등 중요 작업
  if (toolName === "bash" && args.command) {
    const cmd = args.command.toLowerCase();
    if (cmd.includes("git commit") || cmd.includes("git push")) {
      return "high";
    }
    if (cmd.includes("rm -rf") || cmd.includes("drop table")) {
      return "critical";
    }
  }

  // 의사결정 관련
  if (toolName === "question") {
    return "high";
  }

  // 서브태스크 위임
  if (toolName === "task") {
    return "medium";
  }

  return "low";
}

// ==================== ACTION DESCRIPTION ====================
/**
 * Action 설명 생성
 */
function buildActionDescription(
  toolName: string,
  args: Record<string, any>,
): string {
  switch (toolName) {
    case "bash":
      const cmd = args.command || "unknown";
      return `bash 명령 실행: ${cmd.slice(0, 100)}`;

    case "read":
      return `파일 읽기: ${args.filePath || "unknown"}`;

    case "edit":
      return `파일 수정: ${args.filePath || "unknown"}`;

    case "write":
      return `파일 생성: ${args.filePath || "unknown"}`;

    case "task":
      return `서브태스크 실행: ${args.description?.slice(0, 50) || "unknown"}`;

    case "question":
      return `사용자 질문: ${args.questions?.[0]?.header || "unknown"}`;

    case "glob":
      return `파일 검색: ${args.pattern || "unknown"}`;

    case "grep":
      return `내용 검색: ${args.pattern || "unknown"}`;

    default:
      return `도구 실행: ${toolName}`;
  }
}

// ==================== PROMPT INJECTION (Task 1.5) ====================
const METADATA_TOOLS_GUIDANCE = `
## 🧠 메모리 시스템 지침

다음 도구를 적극 활용하여 작업 컨텍스트를 기록하세요:

1. **context_intent** - 지금 바로 호출하세요!
   goal: "현재 작업의 목표"
   user_request_summary: (사용자 요청 요약)
   assumptions: (세운 가정들)

2. **context_decision** - 중요한 선택 시 호출
   라이브러리 선택, 아키텍처 결정, 구현 방식 선택 시

3. **context_learning** - 유용한 정보 발견 시 호출
   프로젝트 특성, 오류 해결 방법 등

이 정보들은 향후 유사 작업에 자동으로 참조됩니다.
`;

// ==================== REMINDER MESSAGE (Task 1.5.1) ====================
const INTENT_REMINDER = `
⚠️ **리마인더**: context_intent를 아직 호출하지 않았습니다.
작업 목표를 기록하면 향후 유사 작업에 자동으로 참조됩니다.
`;

const MAX_REMINDERS = 3;

// ==================== PLUGIN ====================
export const ContextManagerPlugin: Plugin = async ({
  $,
  directory,
  client,
}) => {
  debugLog("Plugin initialized", { directory });

  const homeDir = os.homedir();
  const globalConfigDir = path.join(homeDir, ".config/opencode");

  // Path resolution logic (using shared opencode-memory package)
  const getPaths = () => {
    // 1. Check local project structure
    const localScript = path.join(
      directory,
      ".opencode/shared/opencode-memory/src/opencode_memory/cli.py",
    );
    const localVenv = path.join(directory, "venv/bin/python");
    if (fs.existsSync(localScript) && fs.existsSync(localVenv)) {
      return { script: localScript, python: localVenv };
    }

    // 2. Check global shared opencode-memory package structure
    const sharedScript = path.join(
      globalConfigDir,
      "shared/opencode-memory/src/opencode_memory/cli.py",
    );
    const sharedVenv = path.join(globalConfigDir, "venv/bin/python");
    if (fs.existsSync(sharedScript) && fs.existsSync(sharedVenv)) {
      return { script: sharedScript, python: sharedVenv };
    }

    return null;
  };

  const paths = getPaths();

  const runPython = async (args: string[]): Promise<string> => {
    debugLog("runPython called", { args });

    if (!paths) {
      debugLog("runPython error: paths not found");
      return "Error: Context server not found. Please run install.sh";
    }

    try {
      // Use shared package path if available, otherwise use directory
      const pythonPath = paths.script.includes("shared/opencode-memory")
        ? path.join(globalConfigDir, "shared/opencode-memory/src")
        : directory;

      const result =
        await $`PYTHONPATH=${pythonPath} ${paths.python} ${paths.script} ${[...args]}`
          .cwd(directory)
          .quiet()
          .nothrow();

      if (result.exitCode !== 0) {
        const stderr = result.stderr.toString().trim();
        const stdout = result.stdout.toString().trim();
        debugLog("runPython error", {
          exitCode: result.exitCode,
          stderr,
          stdout,
        });
        return `Error (exit ${result.exitCode}): ${stderr || stdout || "unknown error"}`;
      }
      const output = result.stdout.toString().trim();
      debugLog("runPython success", { args, output }); // Log full output for audit
      return output;
    } catch (error) {
      debugLog("runPython exception", { error: String(error) });
      return `Error: ${error}`;
    }
  };

  // State for auto-start
  let isInitialized = false;

  // Store args from tool.execute.before for use in tool.execute.after
  const pendingToolCalls = new Map<string, any>();
  // Store context to be injected in tool.execute.after
  const pendingContext = new Map<string, string>();
  // Store intent from tool.execute.before for use in tool.execute.after (Task 1.6)
  const pendingIntents = new Map<string, IntentCapture>();
  // Track metadata tool usage per session (Task 1.5.1)
  const metadataToolState = new Map<string, MetadataToolState>();
  // Track problem tracking state per session (Phase 2: Task 2.4, 2.5)
  const problemTrackingState = new Map<string, ProblemTrackingState>();

  /**
   * Get or create metadata tool state for a session
   */
  const getMetadataState = (sessionId: string): MetadataToolState => {
    if (!metadataToolState.has(sessionId)) {
      metadataToolState.set(sessionId, {
        intentCalled: false,
        reminderCount: 0,
        sessionStarted: false,
      });
    }
    return metadataToolState.get(sessionId)!;
  };

  /**
   * Get or create problem tracking state for a session
   */
  const getProblemState = (sessionId: string): ProblemTrackingState => {
    if (!problemTrackingState.has(sessionId)) {
      problemTrackingState.set(sessionId, {
        activeProblemId: null,
        lastErrorOutput: null,
      });
    }
    return problemTrackingState.get(sessionId)!;
  };

  /**
   * Record semantic record asynchronously (fire-and-forget)
   */
  const recordSemanticAsync = async (
    payload: SemanticRecordPayload,
  ): Promise<void> => {
    try {
      const args = ["record-semantic", JSON.stringify(payload)];
      await runPython(args);
      debugLog("Semantic record sent", { intent: payload.intent });
    } catch (error) {
      debugLog("Semantic record error", { error: String(error) });
      // Don't throw - fire and forget
    }
  };

  /**
   * Start problem tracking asynchronously (Phase 2: Task 2.4)
   */
  const startProblemAsync = async (
    sessionId: string,
    errorMessage: string,
  ): Promise<string | null> => {
    try {
      const payload = {
        session_id: sessionId,
        error_message: errorMessage,
      };
      const resultStr = await runPython([
        "problem-start",
        JSON.stringify(payload),
      ]);
      try {
        const result = JSON.parse(resultStr);
        if (result.problem_id) {
          debugLog("Problem tracking started", {
            problemId: result.problem_id,
            errorType: result.error_type,
          });
          return result.problem_id;
        }
      } catch (parseError) {
        debugLog("Failed to parse problem start result", { resultStr });
      }
    } catch (error) {
      debugLog("Start problem error", { error: String(error) });
    }
    return null;
  };

  /**
   * Record problem attempt asynchronously (Phase 2: Task 2.4)
   */
  const addProblemAttemptAsync = async (
    problemId: string,
    solution: string,
  ): Promise<void> => {
    try {
      const payload = {
        problem_id: problemId,
        solution: solution,
      };
      await runPython(["problem-attempt", JSON.stringify(payload)]);
      debugLog("Problem attempt recorded", { problemId, solution });
    } catch (error) {
      debugLog("Add problem attempt error", { error: String(error) });
    }
  };

  /**
   * Resolve problem asynchronously (Phase 2: Task 2.5)
   */
  const resolveProblemAsync = async (
    problemId: string,
    solution: string,
  ): Promise<void> => {
    try {
      const payload = {
        problem_id: problemId,
        solution: solution,
      };
      await runPython(["problem-resolve", JSON.stringify(payload)]);
      debugLog("Problem resolved", { problemId, solution });
    } catch (error) {
      debugLog("Resolve problem error", { error: String(error) });
    }
  };

  return {
    tool: {
      // ==================== EXISTING TOOLS ====================
      context_start: tool({
        description:
          "Start task - Search related long-term memories and initialize Working Memory.",
        args: {
          task: tool.schema.string().describe("Task description"),
        },
        async execute(args) {
          const raw = await runPython(["start", "--task", args.task]);
          isInitialized = true;

          // Task 1.5: 프롬프트 인젝션 - 메타데이터 도구 사용 지침 추가
          const guidance = METADATA_TOOLS_GUIDANCE.replace(
            '"현재 작업의 목표"',
            `"${args.task}"`,
          );

          const formatted = formatContextStartMarkdown(raw);

          return `${formatted}\n\n${guidance}`;
        },
      }),

      context_checkpoint: tool({
        description:
          "Checkpoint - Compress work memories and save to long-term memory.",
        args: {
          summary: tool.schema.string().optional().describe("Progress summary"),
        },
        async execute(args) {
          const cmdArgs = args.summary
            ? ["checkpoint", "--summary", args.summary]
            : ["checkpoint"];
          return await runPython(cmdArgs);
        },
      }),

      context_end: tool({
        description: "End task - Save remaining work memories and cleanup.",
        args: {
          result: tool.schema
            .string()
            .optional()
            .describe("Task result summary"),
        },
        async execute(args) {
          const cmdArgs = args.result
            ? ["end", "--result", args.result]
            : ["end"];
          return await runPython(cmdArgs);
        },
      }),

      context_status: tool({
        description: "Check current context status.",
        args: {
          _placeholder: tool.schema
            .boolean()
            .describe("Placeholder. Always pass true."),
        },
        async execute() {
          return await runPython(["status"]);
        },
      }),

      // ==================== NEW METADATA TOOLS (Task 1.4) ====================
      context_intent: tool({
        description: `현재 작업의 의도와 목표를 기록합니다.
   
**언제 호출?**
- 새 작업 시작 시 (context_start 직후 권장)
- 사용자 요청 받은 직후

**왜 필요?**
이 정보는 장기 기억에 저장되어 향후 유사 작업에 참조됩니다.`,
        args: {
          goal: tool.schema.string().describe("현재 작업의 목표 (1-2문장)"),
          user_request_summary: tool.schema
            .string()
            .optional()
            .describe("사용자 요청 요약"),
          context: tool.schema
            .string()
            .optional()
            .describe("배경 정보, 제약 사항"),
          assumptions: tool.schema
            .array(tool.schema.string())
            .optional()
            .describe("세운 가정들"),
          agent_thoughts: tool.schema
            .array(tool.schema.string())
            .optional()
            .describe("에이전트가 직접 작성한 사고 흐름 리스트"),
          agent_thoughts_text: tool.schema
            .string()
            .optional()
            .describe("줄바꿈으로 구분된 사고 흐름 입력 (배열 대신 사용 가능)"),
        },
        async execute(args) {
          const agentThoughts = normalizeAgentThoughts(
            args.agent_thoughts,
            args.agent_thoughts_text,
          );
          const payload = {
            session_id: "current", // Will be replaced by server with actual session
            goal: args.goal,
            user_request_summary: args.user_request_summary || null,
            context: args.context || null,
            assumptions: args.assumptions || [],
            agent_thoughts: agentThoughts,
          };

          const result = await runPython([
            "record-intent",
            JSON.stringify(payload),
          ]);

          // Mark intent as called for this session
          // Note: We use "current" as a placeholder since sessionID is not available in execute

          return formatJsonOutput(result);
        },
      }),

      context_decision: tool({
        description: `중요한 의사결정을 기록합니다.

**언제 호출?**
- 라이브러리/프레임워크 선택 시
- 아키텍처 패턴 결정 시
- 구현 방식 선택 시`,
        args: {
          decision_type: tool.schema
            .string()
            .describe("결정 유형 (library, architecture, approach)"),
          choice: tool.schema.string().describe("최종 선택"),
          alternatives: tool.schema
            .array(tool.schema.string())
            .optional()
            .describe("고려한 대안들"),
          rationale: tool.schema.string().describe("선택 이유"),
          is_user_preference: tool.schema
            .boolean()
            .optional()
            .describe("사용자 직접 선택 여부"),
        },
        async execute(args) {
          const payload = {
            session_id: "current",
            decision_type: args.decision_type,
            choice: args.choice,
            alternatives: args.alternatives || [],
            rationale: args.rationale,
            is_user_preference: args.is_user_preference || false,
          };

          const result = await runPython([
            "record-decision",
            JSON.stringify(payload),
          ]);

          return formatJsonOutput(result);
        },
      }),

      context_learning: tool({
        description: `작업 중 발견한 학습 사항을 기록합니다.

**언제 호출?**
- 오류 해결 후 교훈 발견 시
- 프로젝트 특성 파악 시
- 유용한 패턴 발견 시`,
        args: {
          learning: tool.schema.string().describe("배운 내용 (1-2문장)"),
          category: tool.schema
            .string()
            .optional()
            .describe("분류 (project, pattern, preference, error_solution)"),
        },
        async execute(args) {
          const payload = {
            session_id: "current",
            learning: args.learning,
            category: args.category || null,
          };

          const result = await runPython([
            "record-learning",
            JSON.stringify(payload),
          ]);

          return formatJsonOutput(result);
        },
      }),
    },
    event: async ({ event }) => {
      debugLog("Event received", { type: event.type });

      // Initialize session asynchronously (non-blocking)
      if (event.type === "session.created") {
        const sessionId = event.data?.id;
        debugLog("session.created - calling init", { sessionId });
        if (sessionId) {
          // Initialize metadata tool state for this session
          metadataToolState.set(sessionId, {
            intentCalled: false,
            reminderCount: 0,
            sessionStarted: false,
          });

          runPython(["init", "--session", sessionId]).catch((e) => {
            debugLog("session.created init error", { error: String(e) });
          });
        }
      }

      // Note: Auto-start is now handled in tool.execute.before
      // message.updated event.data is empty in opencode, so we can't use it
    },
    "tool.execute.before": async (input, _output) => {
      debugLog("tool.execute.before called", {
        tool: input.tool,
        sessionID: input.sessionID,
        callID: input.callID,
        args: _output.args,
        isInitialized,
      });

      // Store args for use in tool.execute.after
      pendingToolCalls.set(input.callID, _output.args);

      // Task 1.6: Intent 캡처 (규칙 기반)
      const intentCapture = captureIntent(input.tool, _output.args || {});
      pendingIntents.set(input.callID, intentCapture);
      debugLog("Intent captured", {
        callID: input.callID,
        intent: intentCapture.intent,
      });

      // Track metadata tool calls (Task 1.5.1)
      if (input.tool === "context_intent") {
        const state = getMetadataState(input.sessionID);
        state.intentCalled = true;
        debugLog("context_intent called - marking state", {
          sessionID: input.sessionID,
        });
      }

      // Auto-start context on first tool execution (if not already initialized)
      // This is more reliable than message.updated since event.data is empty
      if (!isInitialized) {
        const ignoredForAutoStart = new Set([
          "context_start",
          "context_checkpoint",
          "context_end",
          "context_status",
          "context_intent",
          "context_decision",
          "context_learning",
        ]);

        if (!ignoredForAutoStart.has(input.tool)) {
          debugLog("Auto-start triggered from tool.execute.before", {
            tool: input.tool,
          });
          isInitialized = true; // Set immediately to prevent duplicate starts

          // Mark session as started
          const state = getMetadataState(input.sessionID);
          state.sessionStarted = true;

          // Build task description from tool context
          let taskDescription = `Tool execution: ${input.tool}`;
          const args = _output.args || {};

          if (input.tool === "bash" && args.description) {
            taskDescription = args.description;
          } else if (input.tool === "read" && args.filePath) {
            taskDescription = `Reading file: ${args.filePath}`;
          } else if (input.tool === "edit" && args.filePath) {
            taskDescription = `Editing file: ${args.filePath}`;
          } else if (input.tool === "task" && args.description) {
            taskDescription = args.description;
          }

          try {
            const resultStr = await runPython([
              "start",
              "--task",
              taskDescription,
            ]);
            try {
              const result = JSON.parse(resultStr);
              if (result.context_summary) {
                // Add guidance to context
                const guidance = METADATA_TOOLS_GUIDANCE.replace(
                  '"현재 작업의 목표"',
                  `"${taskDescription}"`,
                );
                pendingContext.set(
                  input.callID,
                  result.context_summary + "\n" + guidance,
                );
                debugLog("Context retrieved for injection", {
                  length: result.context_summary.length,
                });
              }
            } catch (parseError) {
              debugLog("Failed to parse start result", {
                error: String(parseError),
                resultStr,
              });
            }
            isInitialized = true; // Set immediately to prevent duplicate starts
          } catch (e) {
            debugLog("Auto-start error", { error: String(e) });
            isInitialized = false; // Reset on failure
          }
        }
      }
    },
    "tool.execute.after": async (input, _output) => {
      const toolName = input.tool;
      const args = pendingToolCalls.get(input.callID) || {};
      pendingToolCalls.delete(input.callID); // Cleanup

      // Get captured intent
      const intentCapture = pendingIntents.get(input.callID);
      pendingIntents.delete(input.callID);

      // Inject context if available
      const contextToInject = pendingContext.get(input.callID);
      pendingContext.delete(input.callID);

      if (contextToInject) {
        _output.output += `\n\n=== Context Memory ===\n${contextToInject}`;
        debugLog("Injected context into tool output", {
          length: contextToInject.length,
        });
      }

      debugLog("tool.execute.after called", {
        tool: toolName,
        sessionID: input.sessionID,
        callID: input.callID,
        hasArgs: !!args,
      });

      // Task 1.5.1: 리마인더 주입
      const state = getMetadataState(input.sessionID);
      if (
        state.sessionStarted &&
        !state.intentCalled &&
        state.reminderCount < MAX_REMINDERS
      ) {
        const ignoredForReminder = new Set([
          "context_start",
          "context_checkpoint",
          "context_end",
          "context_status",
          "context_intent",
          "context_decision",
          "context_learning",
          "read",
          "glob",
          "grep",
        ]);

        if (!ignoredForReminder.has(toolName)) {
          _output.output += INTENT_REMINDER;
          state.reminderCount++;
          debugLog("Intent reminder injected", {
            sessionID: input.sessionID,
            reminderCount: state.reminderCount,
          });
        }
      }

      // Ignored tools for semantic recording
      const ignoredTools = new Set([
        "read",
        "glob",
        "ls",
        "grep",
        "context_start",
        "context_checkpoint",
        "context_end",
        "context_status",
        "context_intent",
        "context_decision",
        "context_learning",
        "context7_resolve-library-id",
        "context7_query-docs",
      ]);

      if (!ignoredTools.has(toolName)) {
        debugLog("Processing tool for semantic record", { tool: toolName });
        try {
          // Task 1.7: Outcome 캡처
          const success = determineSuccess(toolName, _output.output);
          const outcome = captureOutcome(toolName, _output.output, success);
          const action = buildActionDescription(toolName, args);
          const importance = assessImportance(toolName, args, success);

          // Task 1.9: SemanticRecord 전송 (비동기, fire-and-forget)
          const payload: SemanticRecordPayload = {
            session_id: input.sessionID,
            intent: intentCapture?.intent || `도구 실행: ${toolName}`,
            action: action,
            outcome: outcome,
            tool_name: toolName,
            tool_args: args,
            success: success,
            importance: importance,
          };

          // Fire and forget - don't block tool execution
          recordSemanticAsync(payload).catch((e) => {
            debugLog("recordSemanticAsync error", { error: String(e) });
          });

          // ==================== Phase 2: Problem Tracking (Task 2.4, 2.5) ====================
          const problemState = getProblemState(input.sessionID);

          if (!success && detectsError(_output.output)) {
            // 오류 감지 - 문제 추적 시작
            const errorMsg = extractErrorMessage(_output.output);
            if (errorMsg) {
              debugLog("Error detected, starting problem tracking", {
                sessionID: input.sessionID,
                errorMsg,
              });

              startProblemAsync(input.sessionID, errorMsg)
                .then((problemId) => {
                  if (problemId) {
                    problemState.activeProblemId = problemId;
                    problemState.lastErrorOutput = _output.output;
                  }
                })
                .catch((e) => {
                  debugLog("startProblemAsync error", { error: String(e) });
                });
            }
          } else if (success && problemState.activeProblemId) {
            // 성공한 작업이 이전 오류와 관련되어 있으면 해결로 간주
            debugLog("Success after error, attempting to resolve problem", {
              sessionID: input.sessionID,
              problemId: problemState.activeProblemId,
            });

            // 해결 시도 기록 및 문제 해결
            const solution = action;
            resolveProblemAsync(problemState.activeProblemId, solution)
              .then(() => {
                problemState.activeProblemId = null;
                problemState.lastErrorOutput = null;
              })
              .catch((e) => {
                debugLog("resolveProblemAsync error", { error: String(e) });
              });
          } else if (!success && problemState.activeProblemId) {
            // 추가 실패 시도 기록
            addProblemAttemptAsync(
              problemState.activeProblemId,
              action,
            ).catch((e) => {
              debugLog("addProblemAttemptAsync error", { error: String(e) });
            });
          }
          // ==================== End Phase 2 ====================

          // Legacy: Also record to working memory for backward compatibility
          let recordType = "note";
          let content = `Executed tool: ${toolName}`;
          let file = "";

          if (toolName === "write" || toolName === "edit") {
            recordType = "change";
            file = args.filePath || "";
            content = `Modified file: ${file}`;
          } else if (toolName === "bash") {
            content = `Ran command: ${args.command || "unknown"}`;
          } else if (toolName === "task") {
            recordType = "decision";
            content = `Delegated task to subagent: ${args.description || "unknown"}`;
            debugLog("Task tool detected", { description: args.description });
          }

          debugLog("Recording tool execution (legacy)", {
            recordType,
            content,
            file,
          });
          const recordArgs = [
            "record",
            "--type",
            recordType,
            "--content",
            content,
          ];
          if (file) {
            recordArgs.push("--file", file);
          }
          await runPython(recordArgs);

          // Check if checkpoint is needed
          debugLog("Calling auto-checkpoint");
          await runPython(["auto-checkpoint"]);
        } catch (error) {
          debugLog("tool.execute.after error", { error: String(error) });
        }
      } else {
        debugLog("Tool ignored", { tool: toolName });
      }
    },
    "experimental.session.compacting": async (input, output) => {
      debugLog("Compacting session - injecting context", {
        sessionID: input.sessionID,
      });
      try {
        const context = await runPython(["get-compaction-context"]);
        if (
          context &&
          !context.includes("Error retrieving context") &&
          context.trim().length > 0
        ) {
          output.context.push(context);
          debugLog("Context injected into compaction", {
            length: context.length,
            fullContext: context, // Log full context for audit
          });
        } else {
          debugLog("No context to inject or error occurred", { context });
        }
      } catch (error) {
        debugLog("Compaction injection error", { error: String(error) });
      }
    },
  };
};

export default ContextManagerPlugin;
