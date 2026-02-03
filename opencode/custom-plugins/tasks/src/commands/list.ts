// src/commands/list.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskList, ToolResponse, StatusSummary } from '../types';

export interface ListArgs {
  sessionId: string;    // 세션 ID (필수)
  format?: 'markdown' | 'json' | 'table';
}

export interface ListResult {
  success: boolean;
  taskLists: TaskList[];
  formattedOutput: string;
  message: string;
  response: ToolResponse;
}

export async function listCommand(args: ListArgs): Promise<ListResult> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      const emptyTaskList: TaskList = {
        title: 'Empty',
        agent: 'unknown',
        createdAt: new Date().toISOString(),
        sessionId: args.sessionId,
        tasks: []
      };

      const emptyStatus: StatusSummary = {
        agent: 'unknown',
        title: 'Empty',
        total: 0,
        completed: 0,
        inProgress: 0,
        pending: 0,
        completionRate: 0
      };

      const emptyResponse: ToolResponse = {
        title: `List: No task lists found`,
        output: '',
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: 'list',
          message: `No task lists found for session: ${args.sessionId}`
        }
      };

      return {
        success: true,
        taskLists: [],
        formattedOutput: '',
        message: `No task lists found for session: ${args.sessionId}`,
        response: emptyResponse
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

    // 첫 번째 task list를 기준으로 summary 계산
    const primaryTaskList = taskLists[0];
    
    // taskLists가 비어있는 경우 처리 (예: read error)
    if (!primaryTaskList) {
      const emptyResponse: ToolResponse = {
        title: `List: No valid task lists`,
        output: formattedOutput,
        metadata: {
          operation: 'list',
          message: `Found ${taskLists.length} valid task list(s)`
        }
      };

      return {
        success: true,
        taskLists,
        formattedOutput,
        message: `Found ${taskLists.length} valid task list(s)`,
        response: emptyResponse
      };
    }

    let total = 0;
    let completed = 0;
    let inProgress = 0;
    let pending = 0;

    for (const taskList of taskLists) {
      // Flat structure - count only top-level tasks
      for (const task of taskList.tasks) {
        total++;
        if (task.status === 'completed') {
          completed++;
        } else if (task.status === 'in_progress') {
          inProgress++;
        } else {
          pending++;
        }
      }
    }

    const statusSummary: StatusSummary = {
      agent: primaryTaskList.agent,
      title: primaryTaskList.title,
      total,
      completed,
      inProgress,
      pending,
      completionRate: total > 0 ? Math.round((completed / total) * 100) : 0
    };

    // Native UI Response 생성
    const response: ToolResponse = {
      title: `${primaryTaskList.title} (${total} tasks)`,
      output: formattedOutput,
      metadata: {
        taskList: primaryTaskList,
        tasks: primaryTaskList.tasks,
        summary: statusSummary,
        operation: 'list',
        message: `Found ${taskLists.length} task list(s)`
      }
    };

    return {
      success: true,
      taskLists,
      formattedOutput,
      message: `Found ${taskLists.length} task list(s)`,
      response
    };
  } catch (error) {
    throw error;
  }
}
