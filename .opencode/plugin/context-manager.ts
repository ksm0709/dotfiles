/**
 * Context Manager Plugin
 *
 * Plugin for automatic 2-layer context management for agents.
 * - Working Memory: Session-based short-term memory
 * - Context Memory: OpenMemory-based long-term memory
 */

import type { Plugin } from "@opencode-ai/plugin";
import { tool } from "@opencode-ai/plugin";

export const ContextManagerPlugin: Plugin = async ({ $, directory, client }) => {
  // Python backend paths
  const pythonScript = `${directory}/.opencode/shared/context/context_server.py`;
  const pythonExe = `${directory}/venv/bin/python`;

  // Python execution helper using ctx.$ (opencode's shell)
  // Spread args array to pass each element as separate argument
  const runPython = async (args: string[]): Promise<string> => {
    const result = await $`PYTHONPATH=${directory} ${pythonExe} ${pythonScript} ${[...args]}`
      .cwd(directory)
      .quiet()
      .nothrow();
    
    if (result.exitCode !== 0) {
      const stderr = result.stderr.toString().trim();
      const stdout = result.stdout.toString().trim();
      return `Error (exit ${result.exitCode}): ${stderr || stdout || "unknown error"}`;
    }
    return result.stdout.toString().trim();
  };

  await client.app.log({
    service: "context-manager",
    level: "info",
    message: "Context Manager Plugin initialized",
  });

  return {
    tool: {
      context_start: tool({
        description: "Start task - Search related long-term memories and initialize Working Memory.",
        args: {
          task: tool.schema.string().describe("Task description"),
        },
        async execute(args) {
          return await runPython(["start", "--task", args.task]);
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
  };
};

export default ContextManagerPlugin;
