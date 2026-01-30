// src/commands/status.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskStatusSummary, TaskDetail, TaskStatus } from '../types';

export interface StatusArgs {
  sessionId: string;    // 세션 ID (필수)
}

export interface StatusResult {
  success: boolean;
  summaries: TaskStatusSummary[];
  formattedOutput: string;
  message: string;
}

export async function statusCommand(args: StatusArgs): Promise<StatusResult> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      return {
        success: true,
        summaries: [],
        formattedOutput: '',
        message: `No task lists found for session: ${args.sessionId}`
      };
    }

    const summaries: TaskStatusSummary[] = [];
    let formattedOutput = '';

    // Show status for all task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      const stats = calculateStats(taskList.tasks);

      const summary: TaskStatusSummary = {
        agent: taskList.agent,
        title: taskList.title,
        status: stats.status,
        completionRate: stats.completionRate,
        completedCount: stats.completedCount,
        totalCount: stats.totalCount,
        currentPhase: taskList.currentPhase
      };

      summaries.push(summary);
      formattedOutput += formatter.formatStatusSummary(summary) + '\n\n';
    }

    return {
      success: true,
      summaries,
      formattedOutput,
      message: `Found ${summaries.length} task list(s)`
    };
  } catch (error) {
    throw error;
  }
}

function calculateStats(tasks: TaskDetail[]): { status: TaskStatus; completionRate: number; completedCount: number; totalCount: number } {
  let total = 0;
  let completed = 0;
  let inProgress = 0;

  const countTasks = (taskList: TaskDetail[]) => {
    for (const task of taskList) {
      total++;
      if (task.status === 'completed') {
        completed++;
      } else if (task.status === 'in_progress') {
        inProgress++;
      }
      if (task.subtasks) {
        countTasks(task.subtasks);
      }
    }
  };

  countTasks(tasks);

  const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;
  let status: TaskStatus = 'pending';
  if (completed === total && total > 0) {
    status = 'completed';
  } else if (inProgress > 0 || completed > 0) {
    status = 'in_progress';
  }

  return { status, completionRate, completedCount: completed, totalCount: total };
}
