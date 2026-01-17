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

export const ContextManagerPlugin: Plugin = async ({ $, directory, client }) => {
  const homeDir = os.homedir();
  const globalConfigDir = path.join(homeDir, ".config/opencode");

  // Path resolution logic (reflecting global/local structure differences)
  const getPaths = () => {
    // 1. Check local project structure
    const localScript = path.join(directory, ".opencode/shared/context/context_server.py");
    const localVenv = path.join(directory, "venv/bin/python");
    if (fs.existsSync(localScript) && fs.existsSync(localVenv)) {
      return { script: localScript, python: localVenv };
    }

    // 2. Check global config structure (Global doesn't have .opencode prefix for shared)
    const globalScript = path.join(globalConfigDir, "shared/context/context_server.py");
    const globalVenv = path.join(globalConfigDir, "venv/bin/python");
    if (fs.existsSync(globalScript) && fs.existsSync(globalVenv)) {
      return { script: globalScript, python: globalVenv };
    }

    // 3. Fallback for nested structure (just in case)
    const nestedScript = path.join(globalConfigDir, "shared/context/context/context_server.py");
    if (fs.existsSync(nestedScript) && fs.existsSync(globalVenv)) {
      return { script: nestedScript, python: globalVenv };
    }

    return null;
  };

  const paths = getPaths();
  
  const runPython = async (args: string[]): Promise<string> => {
    if (!paths) {
      return "Error: Context server not found. Please run install.sh";
    }
    
    try {
      const result = await $`PYTHONPATH=${directory} ${paths.python} ${paths.script} ${[...args]}`
        .cwd(directory)
        .quiet()
        .nothrow();
      
      if (result.exitCode !== 0) {
        const stderr = result.stderr.toString().trim();
        const stdout = result.stdout.toString().trim();
        return `Error (exit ${result.exitCode}): ${stderr || stdout || "unknown error"}`;
      }
      return result.stdout.toString().trim();
    } catch (error) {
      return `Error: ${error}`;
    }
  };

  // State for auto-start
  let isInitialized = false;

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
      // Initialize session asynchronously (non-blocking)
      if (event.type === "session.created") {
        runPython(["init", "--session", event.data.id]).catch(() => {});
      }

      // Auto-start on first user message
      if (event.type === "message.updated" && !isInitialized) {
        try {
          const message = event.data;
          if (message.role === "user") {
            const text = message.parts
              ?.filter((p: any) => p.type === "text")
              ?.map((p: any) => p.text)
              ?.join("\n");
            
            if (text?.trim()) {
              runPython(["start", "--task", text])
                .then(() => { isInitialized = true; })
                .catch(() => {});
            }
          }
        } catch (error) {
          // ignore errors in auto-start
        }
      }
    },
    "tool.execute.after": async (input, _output) => {
      const toolName = input.tool;
      const ignoredTools = new Set([
        "read", "glob", "ls", "grep", 
        "context_start", "context_checkpoint", "context_end", "context_status",
        "context7_resolve-library-id", "context7_query-docs"
      ]);
      
      if (!ignoredTools.has(toolName)) {
        try {
          // 1. Auto-record the tool execution
          let recordType = "note";
          let content = `Executed tool: ${toolName}`;
          let file = "";

          if (toolName === "write" || toolName === "edit") {
            recordType = "change";
            file = (input.args as any).filePath || "";
            content = `Modified file: ${file}`;
          } else if (toolName === "bash") {
            content = `Ran command: ${(input.args as any).command}`;
          }

          await runPython(["record", "--type", recordType, "--content", content, "--file", file]);

          // 2. Check if checkpoint is needed
          await runPython(["auto-checkpoint"]);
        } catch (error) {
          // ignore errors in auto-checkpoint
        }
      }
    }
  };
};

export default ContextManagerPlugin;
