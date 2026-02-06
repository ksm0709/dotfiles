#!/usr/bin/env node

// src/index.ts
// OpenCode Plugin for tasks management
// Unified single-tool interface for all task operations

import type { Plugin } from "@opencode-ai/plugin";
import { tool } from "@opencode-ai/plugin";
import { unifiedCommand } from './commands/unified';
import { Storage } from './lib/storage';
import { Parser } from './lib/parser';
import { CompletionChecker } from './lib/completion-checker';
import { PromptGenerator } from './lib/prompt-generator';

export const TasksPlugin: Plugin = async ({ client }: { client: any }) => {
  // Storage와 Parser 인스턴스 생성 (재사용)
  const storage = new Storage();
  const parser = new Parser();

  return {
    // Session idle event handler - check for incomplete tasks
    event: async ({ event }: { event: any }) => {
      if (event.type === 'session.idle') {
        // OpenCode event structure: sessionID is in event.properties
        const sessionId = event.properties?.sessionID || 
                         event.properties?.session_id || 
                         event.properties?.sessionId ||
                         event.sessionID || 
                         event.session_id || 
                         event.sessionId;
        
        if (!sessionId) {
          console.error('Session ID not provided in session.idle event');
          console.error('Event structure:', JSON.stringify(event, null, 2));
          return;
        }

        try {
          // 미완료 작업 확인
          const checker = new CompletionChecker(storage, parser);
          const result = await checker.checkIncompleteTasks(sessionId);

          // 미완료 작업이 있으면 프롬프트 생성 및 주입
          if (result.hasIncomplete) {
            const promptGen = new PromptGenerator();
            const prompt = promptGen.generateIncompleteTaskPrompt(
              result.incompleteTasks,
              result.summary
            );

            // 사용자 입력처럼 메시지 주입
            try {
              await client.session.prompt({
                path: { id: sessionId },
                body: { parts: [{ type: 'text', text: prompt }] }
              });
            } catch (promptError) {
              console.error('Failed to inject completion prompt:', promptError);
            }
          }
        } catch (error) {
          console.error('Error handling session.idle event:', error);
        }
      }
    },

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
            // OpenCode ToolContext에서 sessionID 추출 (대문자 ID 사용)
            // sessionID가 없으면 명확하게 실패 - default-session fallback 금지
            if (!ctx.sessionID) {
              throw new Error('Session ID is required from ToolContext but was not provided. Cannot initialize task list without a valid session ID.');
            }
            const sessionId = ctx.sessionID;

            const result = await unifiedCommand({
              sessionId,
              operations: args.operations
            });

            // Return markdown output string
            // OpenCode tool interface requires Promise<string> return type
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
