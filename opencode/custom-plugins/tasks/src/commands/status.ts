// src/commands/status.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskStatusSummary, TaskDetail, TaskStatus, ToolResponse, TaskList, StatusSummary } from '../types';

export interface StatusArgs {
  sessionId: string;    // 세션 ID (필수)
}

export interface StatusResult {
  success: boolean;
  summaries: TaskStatusSummary[];
  formattedOutput: string;
  message: string;
  response: ToolResponse;
}

export async function statusCommand(args: StatusArgs): Promise<StatusResult> {
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
        title: `Status: No task lists found`,
        output: '',
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: 'status',
          message: `No task lists found for session: ${args.sessionId}`
        }
      };

      return {
        success: true,
        summaries: [],
        formattedOutput: '',
        message: `No task lists found for session: ${args.sessionId}`,
        response: emptyResponse
      };
    }

    const summaries: TaskStatusSummary[] = [];
    const taskLists: TaskList[] = [];
    let formattedOutput = '';

    // Show status for all task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      taskLists.push(taskList);
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

    // 첫 번째 task list를 기준으로 summary 계산
    const primaryTaskList = taskLists[0];
    const overallStats = calculateOverallStats(summaries);
    
    const statusSummary: StatusSummary = {
      agent: primaryTaskList.agent,
      title: primaryTaskList.title,
      total: overallStats.total,
      completed: overallStats.completed,
      inProgress: overallStats.inProgress,
      pending: overallStats.pending,
      completionRate: overallStats.completionRate
    };

    // Native UI Response 생성
    const response: ToolResponse = {
      title: `Progress: ${statusSummary.completionRate}% (${statusSummary.completed}/${statusSummary.total})`,
      output: formattedOutput,
      metadata: {
        taskList: primaryTaskList,
        tasks: primaryTaskList.tasks,
        summary: statusSummary,
        operation: 'status',
        message: `Found ${summaries.length} task list(s)`
      }
    };

    return {
      success: true,
      summaries,
      formattedOutput,
      message: `Found ${summaries.length} task list(s)`,
      response
    };
  } catch (error) {
    throw error;
  }
}

function calculateStats(tasks: TaskDetail[]): { status: TaskStatus; completionRate: number; completedCount: number; totalCount: number } {
  const total = tasks.length;
  const completed = tasks.filter(t => t.status === 'completed').length;
  const inProgress = tasks.filter(t => t.status === 'in_progress').length;

  const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;
  let status: TaskStatus = 'pending';
  if (completed === total && total > 0) {
    status = 'completed';
  } else if (inProgress > 0 || completed > 0) {
    status = 'in_progress';
  }

  return { status, completionRate, completedCount: completed, totalCount: total };
}

/**
 * 전체 통계 계산 (여러 task list의 요약)
 */
function calculateOverallStats(summaries: TaskStatusSummary[]): { total: number; completed: number; inProgress: number; pending: number; completionRate: number } {
  let total = 0;
  let completed = 0;
  let inProgress = 0;

  for (const summary of summaries) {
    total += summary.totalCount;
    completed += summary.completedCount;
    // inProgress는 total - completed - pending이므로
    // pending 계산: pending = total - completed - inProgress
    // 여기서는 간단하게 전체에서 completed를 뺀 나머지로 계산
  }

  // pending 계산: 각 task list의 pending 상태 작업
  let pending = 0;
  for (const summary of summaries) {
    if (summary.status === 'pending') {
      pending += summary.totalCount;
    } else if (summary.status === 'in_progress') {
      // in_progress 상태인 경우, completed를 제외한 나머지는 in_progress로 간주
      pending += summary.totalCount - summary.completedCount;
    } else {
      // completed 상태
      pending += summary.totalCount - summary.completedCount;
    }
  }
  
  // inProgress는 전체에서 completed와 pending을 뺀 값
  inProgress = total - completed - pending;
  if (inProgress < 0) inProgress = 0;

  const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;

  return { total, completed, inProgress, pending, completionRate };
}
