// src/lib/formatter.ts

import { TaskList, TaskDetail, TaskStatusSummary } from '../types';

export class Formatter {
  formatAsMarkdown(taskList: TaskList): string {
    const lines: string[] = [];

    lines.push(`# Task List: ${taskList.title}`);
    lines.push('');
    lines.push(`**에이전트**: ${taskList.agent}`);
    lines.push(`**생성일**: ${taskList.createdAt}`);
    lines.push(`**세션 ID**: ${taskList.sessionId}`);
    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push('## 작업 목록');
    lines.push('');

    for (const task of taskList.tasks) {
      lines.push(...this.formatTaskMarkdown(task, 0));
    }

    lines.push('');
    lines.push('---');
    lines.push('');
    lines.push('## 진행 상황 요약');
    lines.push('');
    
    const stats = this.calculateStats(taskList.tasks);
    lines.push(`**현재 단계**: ${taskList.currentPhase || '미정'}`);
    lines.push(`**상태**: ${stats.status}`);
    lines.push(`**완료율**: ${stats.completionRate}% (${stats.completedCount}/${stats.totalCount})`);
    
    if (taskList.memo) {
      lines.push(`**메모**: ${taskList.memo}`);
    }

    return lines.join('\n');
  }

  private formatTaskMarkdown(task: TaskDetail, indent: number): string[] {
    const lines: string[] = [];
    const prefix = '  '.repeat(indent);
    const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
    const statusEmoji = task.status === 'completed' ? '✅' : 
                       task.status === 'in_progress' ? '🔄' : '⏳';
    
    lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${task.title}`);

    for (const detail of task.details) {
      lines.push(`${prefix}  - ${detail}`);
    }

    if (task.subtasks) {
      for (const subtask of task.subtasks) {
        lines.push(...this.formatTaskMarkdown(subtask, indent + 1));
      }
    }

    return lines;
  }

  formatAsJSON(taskList: TaskList): string {
    return JSON.stringify(taskList, null, 2);
  }

  formatAsTable(taskList: TaskList): string {
    const lines: string[] = [];
    
    lines.push(`# ${taskList.title}`);
    lines.push('');
    lines.push('| ID | 상태 | 제목 | 세부사항 |');
    lines.push('|------|--------|------|----------|');

    const formatTaskRow = (task: TaskDetail, indent: string): void => {
      const status = task.status === 'completed' ? '✅ 완료' : 
                    task.status === 'in_progress' ? '🔄 진행중' : '⏳ 대기';
      const details = task.details.join(', ').substring(0, 30);
      const title = indent + task.title;
      
      lines.push(`| ${task.id} | ${status} | ${title} | ${details}${details.length > 30 ? '...' : ''} |`);

      if (task.subtasks) {
        for (const subtask of task.subtasks) {
          formatTaskRow(subtask, indent + '  ');
        }
      }
    };

    for (const task of taskList.tasks) {
      formatTaskRow(task, '');
    }

    return lines.join('\n');
  }

  formatStatusSummary(summary: TaskStatusSummary): string {
    const lines: string[] = [];
    
    lines.push(`📋 Task Status Summary`);
    lines.push('');
    lines.push(`에이전트: ${summary.agent}`);
    lines.push(`작업: ${summary.title}`);
    lines.push(`상태: ${this.getStatusEmoji(summary.status)} ${summary.status}`);
    lines.push(`완료율: ${summary.completionRate}% (${summary.completedCount}/${summary.totalCount})`);
    
    if (summary.currentPhase) {
      lines.push(`현재 단계: ${summary.currentPhase}`);
    }

    // Progress bar
    const barLength = 20;
    const filledLength = Math.round((summary.completionRate / 100) * barLength);
    const bar = '█'.repeat(filledLength) + '░'.repeat(barLength - filledLength);
    lines.push(`진행도: [${bar}] ${summary.completionRate}%`);

    return lines.join('\n');
  }

  private getStatusEmoji(status: string): string {
    switch (status) {
      case 'completed': return '✅';
      case 'in_progress': return '🔄';
      default: return '⏳';
    }
  }

  private calculateStats(tasks: TaskDetail[]): { status: string; completionRate: number; completedCount: number; totalCount: number } {
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
    let status = 'pending';
    if (completed === total && total > 0) {
      status = 'completed';
    } else if (inProgress > 0 || completed > 0) {
      status = 'in_progress';
    }

    return { status, completionRate, completedCount: completed, totalCount: total };
  }
}
