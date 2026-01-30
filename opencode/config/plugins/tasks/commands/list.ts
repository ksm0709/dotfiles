// src/commands/list.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskList } from '../types';

export interface ListArgs {
  sessionId: string;    // 세션 ID (필수)
  format?: 'markdown' | 'json' | 'table';
}

export interface ListResult {
  success: boolean;
  taskLists: TaskList[];
  formattedOutput: string;
  message: string;
}

export async function listCommand(args: ListArgs): Promise<ListResult> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      return {
        success: true,
        taskLists: [],
        formattedOutput: '',
        message: `No task lists found for session: ${args.sessionId}`
      };
    }

    const format = args.format || 'markdown';
    const taskLists: TaskList[] = [];
    let formattedOutput = '';

    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      taskLists.push(taskList);

      switch (format) {
        case 'json':
          formattedOutput += formatter.formatAsJSON(taskList) + '\n';
          break;
        case 'table':
          formattedOutput += formatter.formatAsTable(taskList) + '\n';
          break;
        case 'markdown':
        default:
          formattedOutput += formatter.formatAsMarkdown(taskList) + '\n';
          break;
      }
      
      formattedOutput += '\n---\n';
    }

    return {
      success: true,
      taskLists,
      formattedOutput,
      message: `Found ${taskLists.length} task list(s)`
    };
  } catch (error) {
    throw error;
  }
}
