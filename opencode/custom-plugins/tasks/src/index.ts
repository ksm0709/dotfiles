#!/usr/bin/env node

// src/index.ts
// OpenCode Plugin for tasks management
// Unified single-tool interface for all task operations

import type { Plugin } from "@opencode-ai/plugin";
import { tool } from "@opencode-ai/plugin";
import { unifiedCommand } from './commands/unified';

export const TasksPlugin: Plugin = async ({ client }: { client: any }) => {
  return {
    // Define custom tools that agents can use
    tool: {
      // Tool: tasks - Unified task management tool
      // Replaces all previous individual tools (tasks_init, tasks_add, tasks_update, tasks_complete, tasks_remove, tasks_list, tasks_status)
      tasks: tool({
        description: "Unified task management tool. Supports init, add, update, complete, remove operations. Execute multiple operations in a single call (max 50). Always shows current session's task status after execution. Other sessions' tasks are completely isolated and not visible.",
        args: {
          operations: tool.schema.array(
            tool.schema.object({
              type: tool.schema.enum(
                ['init', 'add', 'update', 'complete', 'remove'],
                { description: "Operation type" }
              ),
              // For 'init' operation
              agent: tool.schema.optional(
                tool.schema.string({ description: "Agent name (required for init)" })
              ),
              // For 'init' and 'add' operations
              title: tool.schema.optional(
                tool.schema.string({ description: "Task or list title (required for init/add)" })
              ),
              // For 'update', 'complete', 'remove' operations
              id: tool.schema.optional(
                tool.schema.string({ description: "Task ID (required for update/complete/remove)" })
              ),
              // For 'add' operation (optional)
              parent: tool.schema.optional(
                tool.schema.string({ description: "Parent task ID for nested tasks (optional)" })
              ),
              // For 'update' operation
              status: tool.schema.optional(
                tool.schema.enum(
                  ['pending', 'in_progress', 'completed'],
                  { description: "New status (required for update)" }
                )
              ),
            }),
            { description: "Array of operations to execute (max 50). Each operation is executed in order. Partial failures are allowed - successful operations are kept even if some fail." }
          ),
        },
        async execute(args: { operations: any[] }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';

            const result = await unifiedCommand({
              sessionId,
              operations: args.operations
            });

            // Return output string for OpenCode
            // OpenCode expects string output, not the full response object
            return result.response.output;
          } catch (error) {
            const errorMessage = error instanceof Error ? error.message : String(error);
            return `❌ 작업 실행 실패: ${errorMessage}`;
          }
        }
      }),
    }
  };
};

export default TasksPlugin;
