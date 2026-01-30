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

export const TasksPlugin: Plugin = async ({ client }) => {
  return {
    // Define custom tools that agents can use
    tool: {
      // Tool: tasks_init - Initialize a new task list
      tasks_init: tool({
        description: "Initialize a new task list from a tasks.md file. Creates a structured task list for tracking progress.",
        args: {
          agent: tool.schema.string({ description: "Agent name (e.g., senior-sw-engineer, py-code-reviewer)" }),
          title: tool.schema.string({ description: "Task list title" }),
          file: tool.schema.string({ description: "Path to the source tasks.md file" }),
        },
        async execute(args: { agent: string; title: string; file: string }, ctx: any) {
          try {
            await initCommand({
              agent: args.agent,
              title: args.title,
              file: args.file
            });
            return `✅ Task list "${args.title}" initialized successfully for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to initialize task list: ${error}`;
          }
        }
      }),

      // Tool: tasks_list - List all tasks
      tasks_list: tool({
        description: "List all tasks for an agent with their current status and progress.",
        args: {
          agent: tool.schema.string({ description: "Agent name" }),
          format: tool.schema.enum(['markdown', 'json', 'table']).optional({ description: "Output format" }),
        },
        async execute(args: { agent: string; format?: 'markdown' | 'json' | 'table' }, ctx: any) {
          try {
            // Capture console output
            const originalLog = console.log;
            let output = '';
            console.log = (...args: any[]) => {
              output += args.join(' ') + '\n';
            };
            
            await listCommand({
              agent: args.agent,
              format: args.format || 'markdown'
            });
            
            console.log = originalLog;
            return output || `✅ Task list retrieved for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to list tasks: ${error}`;
          }
        }
      }),

      // Tool: tasks_update - Update task status
      tasks_update: tool({
        description: "Update the status of a specific task (pending, in_progress, completed).",
        args: {
          agent: tool.schema.string({ description: "Agent name" }),
          id: tool.schema.string({ description: "Task ID (e.g., task-1, task-2)" }),
          status: tool.schema.enum(['pending', 'in_progress', 'completed'], { description: "New status" }),
        },
        async execute(args: { agent: string; id: string; status: 'pending' | 'in_progress' | 'completed' }, ctx: any) {
          try {
            await updateCommand({
              agent: args.agent,
              id: args.id,
              status: args.status
            });
            return `✅ Task "${args.id}" updated to "${args.status}" for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to update task: ${error}`;
          }
        }
      }),

      // Tool: tasks_complete - Mark task as completed
      tasks_complete: tool({
        description: "Mark a task as completed. Shortcut for tasks_update with status=completed.",
        args: {
          agent: tool.schema.string({ description: "Agent name" }),
          id: tool.schema.string({ description: "Task ID to mark as completed" }),
        },
        async execute(args: { agent: string; id: string }, ctx: any) {
          try {
            await completeCommand({
              agent: args.agent,
              id: args.id
            });
            return `✅ Task "${args.id}" marked as completed for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to complete task: ${error}`;
          }
        }
      }),

      // Tool: tasks_add - Add a new task
      tasks_add: tool({
        description: "Add a new task to the task list with optional details.",
        args: {
          agent: tool.schema.string({ description: "Agent name" }),
          title: tool.schema.string({ description: "Task title" }),
          details: tool.schema.array(tool.schema.string()).optional({ description: "List of task details" }),
          parent: tool.schema.string().optional({ description: "Parent task ID (optional, for nested tasks)" }),
        },
        async execute(args: { agent: string; title: string; details?: string[]; parent?: string }, ctx: any) {
          try {
            await addTaskCommand({
              agent: args.agent,
              title: args.title,
              details: args.details ? args.details.join(',') : undefined,
              parent: args.parent
            });
            return `✅ Task "${args.title}" added successfully for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to add task: ${error}`;
          }
        }
      }),

      // Tool: tasks_remove - Remove a task
      tasks_remove: tool({
        description: "Remove a task from the task list.",
        args: {
          agent: tool.schema.string({ description: "Agent name" }),
          id: tool.schema.string({ description: "Task ID to remove" }),
        },
        async execute(args: { agent: string; id: string }, ctx: any) {
          try {
            await removeCommand({
              agent: args.agent,
              id: args.id,
              force: true
            });
            return `✅ Task "${args.id}" removed successfully for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to remove task: ${error}`;
          }
        }
      }),

      // Tool: tasks_status - Get task status summary
      tasks_status: tool({
        description: "Get a summary of task completion status and progress for an agent.",
        args: {
          agent: tool.schema.string({ description: "Agent name" }),
        },
        async execute(args: { agent: string }, ctx: any) {
          try {
            // Capture console output
            const originalLog = console.log;
            let output = '';
            console.log = (...args: any[]) => {
              output += args.join(' ') + '\n';
            };
            
            await statusCommand({
              agent: args.agent
            });
            
            console.log = originalLog;
            return output || `✅ Status retrieved for agent "${args.agent}"`;
          } catch (error) {
            return `❌ Failed to get status: ${error}`;
          }
        }
      }),
    }
  };
};

export default TasksPlugin;
