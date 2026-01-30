// src/commands/status.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskStatusSummary, TaskDetail, TaskStatus } from '../types';

export interface StatusArgs {
  agent: string;
}

export async function statusCommand(args: StatusArgs): Promise<void> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    const files = await storage.listTaskFiles(args.agent);
    
    if (files.length === 0) {
      console.log(`ℹ️ No task lists found for agent: ${args.agent}`);
      return;
    }

    // Show status for all task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.agent, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      const stats = calculateStats(taskList.tasks);

      const summary: TaskStatusSummary = {
        agent: args.agent,
        title: taskList.title,
        status: stats.status,
        completionRate: stats.completionRate,
        completedCount: stats.completedCount,
        totalCount: stats.totalCount,
        currentPhase: taskList.currentPhase
      };

      console.log(formatter.formatStatusSummary(summary));
      console.log('');
    }
  } catch (error) {
    console.error('❌ Failed to get status:', error);
    process.exit(1);
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
