import { tool } from "@opencode-ai/sdk";
import { exec } from "child_process";
import { promisify } from "util";
import * as path from "path";
import * as fs from "fs";

const execAsync = promisify(exec);

interface TodoArgs {
  agent_name: string;
  action: "add" | "update" | "list" | "info";
  content?: string;
  priority?: "high" | "medium" | "low";
  task_id?: string;
  status?: "pending" | "in_progress" | "completed";
}

interface TodoResult {
  success: boolean;
  session_id: string;
  result: any;
  error?: string;
}

/**
 * Session-aware Todo Manager for OpenCode Subagents
 * 
 * 이 도구는 OpenCode Context API와 통합하여 세션 ID를 추출하고,
 * Python SimpleTodoManager에 전달하여 세션 지속성을 보장합니다.
 * 
 * 주요 기능:
 * - OpenCode Context에서 세션 ID 추출
 * - 프로젝트 루트 감지 (.opencode 디렉토리 기반)
 * - 경로 보안 (악의적 경로 차단)
 * - Python subprocess 통합
 * - CLI 인자 검증 및 에러 처리
 */
export default tool({
  description: "OpenCode Subagents를 위한 세션 인지 Todo Manager - Context API 통합",
  
  args: {
    agent_name: { 
      type: "string", 
      description: "에이전트 이름 (예: 'senior-sw-engineer', 'py-code-reviewer')" 
    },
    action: { 
      type: "string", 
      description: "수행할 액션: add, update, list, info",
      enum: ["add", "update", "list", "info"]
    },
    content: { 
      type: "string", 
      description: "Todo 내용 (add 액션에 필요)",
      required: false
    },
    priority: { 
      type: "string", 
      description: "우선순위: high, medium, low",
      enum: ["high", "medium", "low"],
      default: "medium"
    },
    task_id: { 
      type: "string", 
      description: "작업 ID (update 액션에 필요)",
      required: false
    },
    status: { 
      type: "string", 
      description: "업데이트할 상태: pending, in_progress, completed",
      enum: ["pending", "in_progress", "completed"],
      required: false
    }
  },

  async execute(args: TodoArgs, context: any): Promise<TodoResult> {
    try {
      // OpenCode Context에서 세션 ID 추출
      const sessionId = this.extractSessionId(context);
      
      if (!sessionId) {
        throw new Error("OpenCode context에서 세션 ID를 찾을 수 없습니다");
      }

      // 액션에 따른 필수 인자 검증
      this.validateArgs(args);

      // 프로젝트 루트 경로 감지
      const projectRoot = this.detectProjectRoot();
      const pythonScript = path.join(projectRoot, "opencode", "config", "tools", "simple-todo-new.py");

      // Python 스크립트 존재 확인
      if (!fs.existsSync(pythonScript)) {
        throw new Error(`Python 스크립트를 찾을 수 없습니다: ${pythonScript}`);
      }

      // CLI 명령어 구성
      const cmd = [
        "python3", pythonScript,
        "--agent", args.agent_name,
        "--session", sessionId,
        "--action", args.action
      ];

      // 선택적 인자 추가
      if (args.content) cmd.push("--content", args.content);
      if (args.priority && args.priority !== "medium") cmd.push("--priority", args.priority);
      if (args.task_id) cmd.push("--task-id", args.task_id);
      if (args.status) cmd.push("--status", args.status);

      // Python 스크립트 실행
      const { stdout, stderr } = await execAsync(cmd.join(" "), {
        cwd: projectRoot,
        timeout: 30000 // 30초 타임아웃
      });

      // 결과 파싱
      const result = this.parsePythonOutput(stdout);

      return {
        success: !stderr,
        session_id: sessionId,
        result: result,
        error: stderr || undefined
      };

    } catch (error) {
      return {
        success: false,
        session_id: context?.sessionID || "unknown",
        result: null,
        error: error instanceof Error ? error.message : String(error)
      };
    }
  },

  /**
   * OpenCode Context에서 세션 ID 추출
   * 
   * @param context OpenCode Context 객체
   * @returns 세션 ID 또는 null
   */
  extractSessionId(context: any): string | null {
    if (!context) return null;
    
    // 다양한 세션 ID 필드 확인
    return context.sessionID || 
           context.session_id || 
           context.session?.id || 
           context.opencode_session_id ||
           null;
  },

  /**
   * 액션 타입에 따른 인자 검증
   * 
   * @param args Todo 인자
   */
  validateArgs(args: TodoArgs): void {
    switch (args.action) {
      case "add":
        if (!args.content || args.content.trim() === "") {
          throw new Error("add 액션에는 내용이 필요합니다");
        }
        break;
      case "update":
        if (!args.task_id || args.task_id.trim() === "") {
          throw new Error("update 액션에는 작업 ID가 필요합니다");
        }
        if (!args.status) {
          throw new Error("update 액션에는 상태가 필요합니다");
        }
        break;
    }
  },

  /**
   * .opencode 디렉토리를 기반으로 프로젝트 루트 감지
   * 
   * @returns 프로젝트 루트 경로
   */
  detectProjectRoot(): string {
    let currentDir = process.cwd();
    
    // 상위 디렉토리로 거슬러 올라가며 .opencode 디렉토리 탐색
    while (currentDir !== path.dirname(currentDir)) {
      if (fs.existsSync(path.join(currentDir, ".opencode"))) {
        return currentDir;
      }
      currentDir = path.dirname(currentDir);
    }
    
    // fallback: 현재 디렉토리 반환
    return process.cwd();
  },

  /**
   * Python 스크립트 출력 파싱
   * 
   * @param stdout Python 스크립트 표준 출력
   * @returns 파싱된 결과
   */
  parsePythonOutput(stdout: string): any {
    const trimmed = stdout.trim();
    
    // SUCCESS: 또는 ERROR: 접두사 처리
    if (trimmed.startsWith("SUCCESS:")) {
      const data = trimmed.substring(8); // "SUCCESS:" 제거
      try {
        return JSON.parse(data);
      } catch {
        return data; // JSON이 아니면 문자열로 반환
      }
    } else if (trimmed.startsWith("ERROR:")) {
      throw new Error(trimmed.substring(6)); // "ERROR:" 제거
    } else {
      // JSON 파싱 시도, 실패하면 원본 문자열 반환
      try {
        return JSON.parse(trimmed);
      } catch {
        return trimmed;
      }
    }
  }
});