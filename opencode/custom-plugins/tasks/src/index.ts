#!/usr/bin/env node

// src/index.ts
// OpenCode Plugin for tasks management
// Provides custom tools for task tracking and management

import type { Plugin } from "@opencode-ai/plugin";
import { tool } from "@opencode-ai/plugin";
import { initCommand } from './commands/init';
import { listCommand } from './commands/list';
import { updateCommand } from './commands/update';
import { completeCommand } from './commands/complete';
import { removeCommand } from './commands/remove';
import { statusCommand } from './commands/status';
import { addTaskCommand } from './commands/add-task';
import { batchCommand } from './commands/batch';

export const TasksPlugin: Plugin = async ({ client }: { client: any }) => {
  return {
    // Define custom tools that agents can use
    tool: {
      // Tool: tasks_init - Initialize a new task list
      tasks_init: tool({
        description: "Initialize a new task list. Creates a structured task list for tracking progress. Returns formatted markdown output with current status.",
        args: {
          agent: tool.schema.string({ description: "Agent name (e.g., senior-sw-engineer, py-code-reviewer)" }),
          title: tool.schema.string({ description: "Task list title" }),
        },
        async execute(args: { agent: string; title: string }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await initCommand({
              sessionId,
              agent: args.agent,
              title: args.title
            });
            
            // Return Native UI Response for OpenCode
            return result.response;
          } catch (error) {
            return `❌ Failed to initialize task list: ${error}`;
          }
        }
      }),

      // Tool: tasks_list - List all tasks
      tasks_list: tool({
        description: "List all tasks for current session with their current status and progress. Returns formatted markdown output.",
        args: {
          format: tool.schema.enum(['markdown', 'json', 'table'], { description: "Output format (default: markdown)" }),
        },
        async execute(args: { format?: 'markdown' | 'json' | 'table' }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await listCommand({
              sessionId,
              format: args.format || 'markdown'
            });
            
            if (result.success && result.response) {
              return result.response;
            } else {
              return {
                title: 'List: Error',
                output: result.message,
                metadata: {
                  operation: 'list',
                  message: result.message
                }
              };
            }
          } catch (error) {
            return `❌ Failed to list tasks: ${error}`;
          }
        }
      }),

      // Tool: tasks_update - Update task status
      tasks_update: tool({
        description: "Update the status of a specific task (pending, in_progress, completed). Returns formatted markdown output with current status.",
        args: {
          id: tool.schema.string({ description: "Task ID (e.g., 1, 2, 2.1)" }),
          status: tool.schema.enum(['pending', 'in_progress', 'completed'], { description: "New status" }),
        },
        async execute(args: { id: string; status: 'pending' | 'in_progress' | 'completed' }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await updateCommand({
              sessionId,
              id: args.id,
              status: args.status
            });
            
            // Return Native UI Response for OpenCode
            return result.response;
          } catch (error) {
            return `❌ Failed to update task: ${error}`;
          }
        }
      }),

      // Tool: tasks_complete - Mark task as completed
      tasks_complete: tool({
        description: "Mark a task as completed. Shortcut for tasks_update with status=completed. Returns formatted markdown output with current status.",
        args: {
          id: tool.schema.string({ description: "Task ID to mark as completed" }),
        },
        async execute(args: { id: string }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await completeCommand({
              sessionId,
              id: args.id
            });
            
            // Return Native UI Response for OpenCode
            return result.response;
          } catch (error) {
            return `❌ Failed to complete task: ${error}`;
          }
        }
      }),

      // Tool: tasks_add - Add a new task
      tasks_add: tool({
        description: "Add a new task to the task list with optional details. Returns formatted markdown output with current status.",
        args: {
          title: tool.schema.string({ description: "Task title" }),
          parent: tool.schema.string({ description: "Parent task ID (optional, for nested tasks)" }),
        },
        async execute(args: { title: string; parent?: string }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await addTaskCommand({
              sessionId,
              title: args.title,
              parent: args.parent
            });
            
            // Return Native UI Response for OpenCode
            return result.response;
          } catch (error) {
            return `❌ Failed to add task: ${error}`;
          }
        }
      }),

      // Tool: tasks_remove - Remove a task
      tasks_remove: tool({
        description: "Remove a task from the task list. Returns formatted markdown output with current status.",
        args: {
          id: tool.schema.string({ description: "Task ID to remove" }),
        },
        async execute(args: { id: string }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await removeCommand({
              sessionId,
              id: args.id,
              force: true
            });
            
            // Return Native UI Response for OpenCode
            return result.response;
          } catch (error) {
            return `❌ Failed to remove task: ${error}`;
          }
        }
      }),

      // Tool: tasks_status - Get task status summary
      tasks_status: tool({
        description: "Get a summary of task completion status and progress for current session. Returns formatted markdown output.",
        args: {},
        async execute(args: {}, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await statusCommand({
              sessionId
            });
            
            if (result.success && result.response) {
              return result.response;
            } else {
              return {
                title: 'Status: Error',
                output: result.message,
                metadata: {
                  operation: 'status',
                  message: result.message
                }
              };
            }
          } catch (error) {
            return `❌ Failed to get status: ${error}`;
          }
        }
      }),

      // Tool: tasks_batch - Execute multiple operations in batch
      tasks_batch: tool({
        description: "Execute multiple task operations (add, update, complete, remove) in a single batch call. Maximum 50 operations. Returns formatted markdown output with current status.",
        args: {
          operations: tool.schema.array(
            tool.schema.object({
              type: tool.schema.enum(['add', 'update', 'complete', 'remove'], { description: "Operation type" }),
              id: tool.schema.optional(tool.schema.string({ description: "Task ID (required for update, complete, remove)" })),
              title: tool.schema.optional(tool.schema.string({ description: "Task title (required for add)" })),
              parent: tool.schema.optional(tool.schema.string({ description: "Parent task ID (optional, for nested tasks)" })),
              status: tool.schema.optional(tool.schema.enum(['pending', 'in_progress', 'completed'], { description: "New status (required for update)" })),
            }),
            { description: "Array of operations to execute (max 50)" }
          ),
        },
        async execute(args: { operations: any[] }, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await batchCommand({
              sessionId,
              operations: args.operations
            });
            
            // Return Native UI Response for OpenCode
            return result.response;
          } catch (error) {
            return `❌ Failed to execute batch operations: ${error}`;
          }
        }
      }),
    }
  };
};

export default TasksPlugin;
