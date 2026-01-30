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

export const TasksPlugin: Plugin = async ({ client }: { client: any }) => {
  return {
    // Define custom tools that agents can use
    tool: {
      // Tool: tasks_init - Initialize a new task list
      tasks_init: tool({
        description: "Initialize a new task list. Creates a structured task list for tracking progress.",
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
            
            let response = `✅ Task list "${result.title}" initialized successfully for agent "${result.agent}"\n`;
            response += `📁 File: ${result.fileName}.md\n`;
            response += `📊 Total tasks: ${result.totalTasks}`;
            
            if (result.taskIds.length > 0) {
              response += `\n📝 Available task IDs: ${result.taskIds.join(', ')}`;
            }
            
            return response;
          } catch (error) {
            return `❌ Failed to initialize task list: ${error}`;
          }
        }
      }),

      // Tool: tasks_list - List all tasks
      tasks_list: tool({
        description: "List all tasks for current session with their current status and progress.",
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
            
            if (result.success && result.formattedOutput) {
              return result.formattedOutput;
            } else {
              return result.message;
            }
          } catch (error) {
            return `❌ Failed to list tasks: ${error}`;
          }
        }
      }),

      // Tool: tasks_update - Update task status
      tasks_update: tool({
        description: "Update the status of a specific task (pending, in_progress, completed).",
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
            
            if (result.success) {
              return `✅ ${result.message}`;
            } else {
              return `❌ ${result.message}`;
            }
          } catch (error) {
            return `❌ Failed to update task: ${error}`;
          }
        }
      }),

      // Tool: tasks_complete - Mark task as completed
      tasks_complete: tool({
        description: "Mark a task as completed. Shortcut for tasks_update with status=completed.",
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
            
            if (result.success) {
              return `✅ ${result.message}`;
            } else {
              return `❌ ${result.message}`;
            }
          } catch (error) {
            return `❌ Failed to complete task: ${error}`;
          }
        }
      }),

      // Tool: tasks_add - Add a new task
      tasks_add: tool({
        description: "Add a new task to the task list with optional details.",
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
            
            if (result.success) {
              return `✅ ${result.message}`;
            } else {
              return `❌ ${result.message}`;
            }
          } catch (error) {
            return `❌ Failed to add task: ${error}`;
          }
        }
      }),

      // Tool: tasks_remove - Remove a task
      tasks_remove: tool({
        description: "Remove a task from the task list.",
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
            
            if (result.success) {
              return `✅ ${result.message}`;
            } else {
              return `❌ ${result.message}`;
            }
          } catch (error) {
            return `❌ Failed to remove task: ${error}`;
          }
        }
      }),

      // Tool: tasks_status - Get task status summary
      tasks_status: tool({
        description: "Get a summary of task completion status and progress for current session.",
        args: {},
        async execute(args: {}, ctx: any) {
          try {
            // Extract sessionId from OpenCode context
            const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
            
            const result = await statusCommand({
              sessionId
            });
            
            if (result.success && result.formattedOutput) {
              return result.formattedOutput;
            } else {
              return result.message;
            }
          } catch (error) {
            return `❌ Failed to get status: ${error}`;
          }
        }
      }),
    }
  };
};

export default TasksPlugin;
