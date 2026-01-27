/**
 * Context Manager Plugin
 *
 * Plugin for automatic 2-layer context management for agents.
 */

import type { Plugin } from "@opencode-ai/plugin";
import { tool } from "@opencode-ai/plugin";
import * as fs from "node:fs";
import * as path from "node:path";
import * as os from "node:os";

// ==================== DEBUG LOGGING ====================
const DEBUG = true;
const LOG_FILE = path.join(os.homedir(), ".local/share/opencode/log/context-manager-debug.log");

function debugLog(message: string, data?: any) {
  if (!DEBUG) return;
  const timestamp = new Date().toISOString();
  const logEntry = `[${timestamp}] ${message}${data ? `: ${JSON.stringify(data, null, 2)}` : ''}\n`;
  try {
    fs.appendFileSync(LOG_FILE, logEntry);
  } catch (e) {
    // ignore write errors
  }
}
// ========================================================

export const ContextManagerPlugin: Plugin = async ({ $, directory, client }) => {
  debugLog("Plugin initialized", { directory });
  
  const homeDir = os.homedir();
  const globalConfigDir = path.join(homeDir, ".config/opencode");

  // Path resolution logic (using shared opencode-memory package)
  const getPaths = () => {
    // 1. Check local project structure
    const localScript = path.join(directory, ".opencode/shared/context/context_server.py");
    const localVenv = path.join(directory, "venv/bin/python");
    if (fs.existsSync(localScript) && fs.existsSync(localVenv)) {
      return { script: localScript, python: localVenv };
    }

    // 2. Check global shared opencode-memory package structure
    const sharedScript = path.join(globalConfigDir, "shared/opencode-memory/src/opencode_memory/server.py");
    const sharedVenv = path.join(globalConfigDir, "venv/bin/python");
    if (fs.existsSync(sharedScript) && fs.existsSync(sharedVenv)) {
      return { script: sharedScript, python: sharedVenv };
    }

    // 3. Check legacy global config structure
    const globalScript = path.join(globalConfigDir, "shared/context/context_server.py");
    if (fs.existsSync(globalScript) && fs.existsSync(sharedVenv)) {
      return { script: globalScript, python: sharedVenv };
    }

    // 4. Fallback for nested structure (just in case)
    const nestedScript = path.join(globalConfigDir, "shared/context/context/context_server.py");
    if (fs.existsSync(nestedScript) && fs.existsSync(sharedVenv)) {
      return { script: nestedScript, python: sharedVenv };
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
      
      const result = await $`PYTHONPATH=${pythonPath} ${paths.python} ${paths.script} ${[...args]}`
        .cwd(directory)
        .quiet()
        .nothrow();
      
      if (result.exitCode !== 0) {
        const stderr = result.stderr.toString().trim();
        const stdout = result.stdout.toString().trim();
        debugLog("runPython error", { exitCode: result.exitCode, stderr, stdout });
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

  return {
    tool: {
      context_start: tool({
        description: "Start task - Search related long-term memories and initialize Working Memory.",
        args: {
          task: tool.schema.string().describe("Task description"),
        },
        async execute(args) {
          const result = await runPython(["start", "--task", args.task]);
          isInitialized = true;
          return result;
        },
      }),

      context_checkpoint: tool({
        description: "Checkpoint - Compress work memories and save to long-term memory.",
        args: {
          summary: tool.schema.string().optional().describe("Progress summary"),
        },
        async execute(args) {
          const cmdArgs = args.summary ? ["checkpoint", "--summary", args.summary] : ["checkpoint"];
          return await runPython(cmdArgs);
        },
      }),

      context_end: tool({
        description: "End task - Save remaining work memories and cleanup.",
        args: {
          result: tool.schema.string().optional().describe("Task result summary"),
        },
        async execute(args) {
          const cmdArgs = args.result ? ["end", "--result", args.result] : ["end"];
          return await runPython(cmdArgs);
        },
      }),

      context_status: tool({
        description: "Check current context status.",
        args: {},
        async execute() {
          return await runPython(["status"]);
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
        isInitialized
      });
      
      // Store args for use in tool.execute.after
      pendingToolCalls.set(input.callID, _output.args);
      
      // Auto-start context on first tool execution (if not already initialized)
      // This is more reliable than message.updated since event.data is empty
      if (!isInitialized) {
        const ignoredForAutoStart = new Set([
          "context_start", "context_checkpoint", "context_end", "context_status"
        ]);
        
        if (!ignoredForAutoStart.has(input.tool)) {
          debugLog("Auto-start triggered from tool.execute.before", { tool: input.tool });
          isInitialized = true; // Set immediately to prevent duplicate starts
          
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
            const resultStr = await runPython(["start", "--task", taskDescription]);
            try {
              const result = JSON.parse(resultStr);
              if (result.context_summary) {
                pendingContext.set(input.callID, result.context_summary);
                debugLog("Context retrieved for injection", { length: result.context_summary.length });
              }
            } catch (parseError) {
              debugLog("Failed to parse start result", { error: String(parseError), resultStr });
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
      
      // Inject context if available
      const contextToInject = pendingContext.get(input.callID);
      pendingContext.delete(input.callID);

      if (contextToInject) {
        _output.output += `\n\n=== Context Memory ===\n${contextToInject}`;
        debugLog("Injected context into tool output", { length: contextToInject.length });
      }
      
      debugLog("tool.execute.after called", { 
        tool: toolName, 
        sessionID: input.sessionID,
        callID: input.callID,
        hasArgs: !!args
      });
      
      const ignoredTools = new Set([
        "read", "glob", "ls", "grep", 
        "context_start", "context_checkpoint", "context_end", "context_status",
        "context7_resolve-library-id", "context7_query-docs"
      ]);
      
      if (!ignoredTools.has(toolName)) {
        debugLog("Processing tool", { tool: toolName });
        try {
          // 1. Auto-record the tool execution
          let recordType = "note";
          let content = `Executed tool: ${toolName}`;
          let file = "";

          if (toolName === "write" || toolName === "edit") {
            recordType = "change";
            file = args.filePath || "";
            content = `Modified file: ${file}`;
          } else if (toolName === "bash") {
            content = `Ran command: ${args.command || 'unknown'}`;
          } else if (toolName === "task") {
            recordType = "decision";
            content = `Delegated task to subagent: ${args.description || 'unknown'}`;
            debugLog("Task tool detected", { description: args.description });
          }

          debugLog("Recording tool execution", { recordType, content, file });
          const recordArgs = ["record", "--type", recordType, "--content", content];
          if (file) {
            recordArgs.push("--file", file);
          }
          await runPython(recordArgs);

          // 2. Check if checkpoint is needed
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
      debugLog("Compacting session - injecting context", { sessionID: input.sessionID });
      try {
        const context = await runPython(["get-compaction-context"]);
        if (context && !context.includes("Error retrieving context") && context.trim().length > 0) {
          output.context.push(context);
          debugLog("Context injected into compaction", { 
            length: context.length,
            fullContext: context // Log full context for audit
          });
        } else {
          debugLog("No context to inject or error occurred", { context });
        }
      } catch (error) {
        debugLog("Compaction injection error", { error: String(error) });
      }
    }
  };
};

export default ContextManagerPlugin;
